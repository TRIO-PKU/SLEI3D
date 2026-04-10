import random, rospy, time
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List
from mpl_toolkits.mplot3d import Axes3D
from decision_making.decision_making_voxel_map import DecisionMakingVoxelMap as DMVM
from decision_making.decision_making_data_structure import (
    Decision,
    DecisionMakingData,
    ExpData,
    InsData,
    Meeting,
    InspectionPoint,
    AStarPath,
    PointWithTimeWindow
)
id = str
point = Tuple[float, float, float]
time_window = Tuple[float, float]

# 定义个体类，代表种群中的一个个体
class Individual:

    def __init__(self, genes: List[Meeting], exp_position: point, dmvm: DMVM, inspection_time: float):
        self.dmvm = dmvm  # DecisionMakingVoxelMap
        self.inspection_time = inspection_time  # 每个inspection_point的检测时间 
        self.genes = genes  # 一个个体的基因为，和此次决策中的所有ins的meeting地点组成的列表
        self.exp_position = exp_position  # exp的位置   
        self.fitness = self.calculate_fitness()  # 个体的适应度

    def calculate_fitness(self):
        # 设定约束变量，规定exp必须在时间窗口内到达
        cost_variable = 1
        waste_time: float = 0 # 这个方案浪费的时间
        prev_meeting: Meeting = Meeting() # 上一个meeting
        prev_meeting.meeting_point = self.exp_position
        prev_meeting.time_window = (rospy.Time.now().to_sec(), rospy.Time.now().to_sec())
        prev_arrive_time: float = rospy.Time.now().to_sec()
        for i in range(len(self.genes)):
            meeting = self.genes[i]
            # 1. call A*算法/直线距离，计算exp到达meeting_point的实际时间
            # astar_path: AStarPath | None = self.dmvm.astar_find_path(prev_meeting.meeting_point, meeting.meeting_point)
            # if astar_path == None:
            #     return 0
            # travel_time: float = astar_path.travle_time(1.0, 2.0)
            distance: float = ((meeting.meeting_point[0] - prev_meeting.meeting_point[0])**2 + (meeting.meeting_point[1] - prev_meeting.meeting_point[1])**2 + (meeting.meeting_point[2] - prev_meeting.meeting_point[2])**2)**0.5
            travel_time: float = self.calculate_time_according_to_distance(distance, 1.0, 2.0)
            # 2. 计算exp_travel
            exp_travel: float = travel_time
            meeting.predicted_arrive_time = prev_arrive_time + travel_time
            # 3. 计算exp_wait
            exp_wait: float = max(0,  meeting.time_window[0] - meeting.predicted_arrive_time)
            # 4. 计算ins_wait
            ins_wait: float = 0
            # 若time_window[1]极大，则ins_wait = 0
            if meeting.time_window[1] == float("inf"):   # TODO: if time_window[1] == float('inf')
                if meeting.time_window[0] + self.inspection_time < rospy.Time.now().to_sec():
                    meeting.time_window = (rospy.Time.now().to_sec(), float("inf"))
                    ins_wait = meeting.predicted_arrive_time - rospy.Time.now().to_sec()
                else:
                    ins_wait = max(0, meeting.predicted_arrive_time - (meeting.time_window[0] + self.inspection_time))
            else:
                ins_wait = max(0, meeting.predicted_arrive_time - meeting.time_window[1])
            if meeting.time_window[0] < 10000:
                rospy.logerr_throttle(1, f"Invalid meeting.time_window: {meeting.time_window}")
            # 5. 计算cost和waste_time
            cost_variable = cost_variable*self.cost(self.genes[i], meeting.predicted_arrive_time)
            waste_time += exp_travel + exp_wait + ins_wait # BUG ins_wait too big !!!
            # 6. 更新prev_meeting, prev_arrive_time
            prev_meeting = meeting
            prev_arrive_time = meeting.predicted_arrive_time
        if waste_time == 0:
            return (1 / 0.01) * cost_variable
        else:
            return  (1 / waste_time) * cost_variable
    
    # 约束函数
    def cost(self, meeting: Meeting, predicted_arrive_time: float):
        # predicted_arrive_time: exp到达meeting_point的时间
        # 若exp在time_window内到达meeting_point，则cost为1
        # 否则，cost为0
        if predicted_arrive_time <= meeting.time_window[1]:
            return 1
        return 0

    def calculate_time_according_to_distance(self, distance:float, acc: float, max_v: float) -> float:
        t_acc = max_v / acc  # Time to accelerate to max velocity
        d_acc = 0.5 * acc * t_acc ** 2  # Distance covered during acceleration

        if distance < 2 * d_acc:  # Never reach max velocity
            t_total = (2 * distance / acc) ** 0.5
        else:
            d_const = distance - 2 * d_acc  # Distance covered at constant velocity
            t_const = d_const / max_v  # Time at constant velocity
            t_total = 2 * t_acc + t_const

        return t_total

class GA:
    def __init__(self, dm: DecisionMakingData, ins_meeting_order: Tuple[id], dmvm: DMVM, num_generations: int, population_size: int, decision: Decision, inspection_time: float):
        self.dm = dm    # DecisionMakingData 
        self.inspection_time = inspection_time  # 每个inspection_point的检测时间 
        self.ins_meeting_order: Tuple[id] = ins_meeting_order  # ins的meeting顺序
        self.dmvm = dmvm # DecisionMakingVoxelMap
        self.num_generations: int = num_generations # 进化代数 
        self.population_size: int = self.set_population_size(population_size)  # 种群大小 
        self.mutation_rate: float = 0.01  # 变异概率
        self.already_visited: List[point] = []  # 已经访问过的meeting_point
        self.decision: Decision = decision  # 本次决策
    
    def set_population_size(self, population_size: int):
        max_inspection_points_num: int = 0
        for ins_id in self.ins_meeting_order:
            ins_data: InsData = self.find_ins_data(ins_id)
            max_inspection_points_num = max(max_inspection_points_num, len(ins_data.inspection_points))
        if population_size < max_inspection_points_num:
            return max_inspection_points_num
        else:
            return population_size
    
    # 初始化种群
    def initialize_population(self, time_log):
        gene_length = len(self.ins_meeting_order)
        initial_population: List[Individual] = []

        for i in range(self.population_size):
            genes = []
            for j in range(gene_length):
                ins_id = self.ins_meeting_order[j]
                print("before find ins data"+ str(time.time() - time_log))
                ins_data: InsData = self.find_ins_data(ins_id)    
                ip: InspectionPoint = ins_data.inspection_points[i%len(ins_data.inspection_points)]
                
                meeting = Meeting()
                meeting.ins_id = ins_data.id
                print("before random pick"+ str(time.time() - time_log))
                meeting.meeting_point = self.random_pick(ip, ip.sampling_meeting_points) # the meeting point cannot be duplicated
                meeting.ins_inspection_point = ip.position
                meeting.time_window = ip.time_window
                meeting.allocated_features = []
                genes.append(meeting)
            exp_point: point = self.dm.exp_data.current_position
            individual = Individual(genes, exp_point, self.dmvm, self.inspection_time)
            initial_population.append(individual)
        return initial_population
    
    # 从存储所有ins_data的列表中找到对应的ins_data
    def find_ins_data(self, ins_id: id):    
        for ins_data in self.dm.ins_data_list:
            if ins_data.id == ins_id:
                return ins_data
        return None
    
    def random_pick(self, last_inspection_point: InspectionPoint, sampling_meeting_points: List[PointWithTimeWindow]) -> PointWithTimeWindow:
        # 将meeting_point初始化为原本的位置，防止sampling_meeting_points为空
        meeting_point: PointWithTimeWindow = PointWithTimeWindow(last_inspection_point.position[:3], last_inspection_point.position, last_inspection_point.time_window)
        # 从sampling_meeting_points中随机选择一个meeting_point
        if len(sampling_meeting_points) == 0:
            return meeting_point
        else:
            meeting_point = random.choice(sampling_meeting_points)
            return meeting_point

    # 选择过程
    def selection(self, population, num_parents):
        # 根据适应度排序，选择适应度最高的个体作为父母
        # population: 当前种群
        # num_parents: 选择的父母数量
        sorted_population = sorted(population, key=lambda x: x.fitness, reverse=True)
        return sorted_population[:num_parents]

    # 交叉过程
    def crossover(self, parent1: Individual, parent2: Individual):
        # 单点交叉
        # parent1, parent2: 选择的两个父本个体
        # 随机选择交叉点，交换父本基因，生成两个子代
        if len(parent1.genes) <= 1 or len(parent2.genes) <= 1:
            return parent1, parent2
        point = random.randint(1, len(parent1.genes) - 1)
        child1_genes = parent1.genes[:point] + parent2.genes[point:]
        child2_genes = parent2.genes[:point] + parent1.genes[point:]
        return Individual(child1_genes, self.dm.exp_data.current_position, self.dmvm, self.inspection_time), Individual(child2_genes, self.dm.exp_data.current_position, self.dmvm, self.inspection_time)

    # 变异过程
    def mutation(self, individual: Individual):
        # 对个体的基因序列进行随机变异
        for i in range(len(individual.genes)):
            if random.random() < self.mutation_rate:
                # 获取对应的ins_id的所有sampling_meeting_points
                ins_id = individual.genes[i].ins_id
                all_sampling_meeting_points, last_inspection_point = self.find_ins_all_sampled_point(ins_id)
                mutated_meeting_point: PointWithTimeWindow = self.random_pick(last_inspection_point, all_sampling_meeting_points)
                individual.genes[i].meeting_point = mutated_meeting_point 
                individual.genes[i].ins_inspection_point = mutated_meeting_point.inspection_point
                individual.genes[i].time_window = mutated_meeting_point.time_window # type: ignore
        # 更新个体的适应度
        individual.fitness = individual.calculate_fitness()
        return individual
    
    # 找到对应ins_id的所有sampling_meeting_points和这个ins的最后一个拍照点
    def find_ins_all_sampled_point(self, ins_id: id):
        ins_data: InsData = self.find_ins_data(ins_id)
        last_inspection_point: InspectionPoint = ins_data.inspection_points[-1]
        all_sampling_meeting_points = []
        for ip in ins_data.inspection_points:
            all_sampling_meeting_points.extend(ip.sampling_meeting_points)
        return all_sampling_meeting_points, last_inspection_point
    
    def update_decision(self, best_individual: Individual):
        # 更新决策
        for i in range(len(best_individual.genes)):
            self.decision.meeting_list.append(best_individual.genes[i])
        if best_individual.fitness == 0:
            self.decision.ga_time_cost = float("inf")
        else:
            self.decision.ga_time_cost = 1/best_individual.fitness 
        return self.decision
    
    def update_best_individual(self, best_individual: Individual, best_individual_per_generation: Individual):
        # 更新每一代中适应度最高的个体
        if best_individual == None:
            best_individual = best_individual_per_generation
        elif best_individual.fitness < best_individual_per_generation.fitness:
            best_individual = best_individual_per_generation
        return best_individual

    # 遗传算法主函数
    def genetic_algorithm(self, time_log):
        # population_size: 种群大小
        # num_generations: 进化代数
        # 初始化种群
        print("before initialize population"+ str(time.time() - time_log)) # time_cost: 0.004-0.01
        population = self.initialize_population(time_log)
        best_individual: Individual = None
        best_fitness_per_generation = []
        print("after initialize population"+ str(time.time() - time_log))
        for i in range(self.num_generations):
            # rospy.logwarn(f"第{i}代")
            # 选择
            print("before selection"+ str(time.time() - time_log))
            parents = self.selection(population, self.population_size // 2)  
            # rospy.logwarn(f"本次parents适应度: {[parent.fitness for parent in parents]}")
            next_generation = []
            # 将前两个父本直接加入下一代
            next_generation.extend(parents[:2])
            # 生成新一代
            print("before generate new generation"+ str(time.time() - time_log)) # time_cost: 0.001 - 0.003
            while len(next_generation) < self.population_size:
                parent1, parent2 = random.sample(parents, 2) # TODO：不能重复选择亲本？
                child1, child2 = self.crossover(parent1, parent2)
                child1 = self.mutation(child1)
                child2 = self.mutation(child2)
                next_generation.extend([child1, child2])
                # rospy.logwarn(f"本次child1适应度: {child1.fitness}")
                # rospy.logwarn(f"本次child2适应度: {child2.fitness}")
            population = next_generation
            # 每一代选出适应度最高的个体
            best_individual_per_generation = max(population, key=lambda x: x.fitness)
            best_individual = self.update_best_individual(best_individual, best_individual_per_generation)
            best_fitness_per_generation.append(best_individual_per_generation.fitness)
            # rospy.logwarn(f"本次最优适应度: {best_individual_per_generation.fitness}")
        self.decision: Decision = self.update_decision(best_individual)

        # 可视化每一代的最优适应度
        # plt.plot(range(self.num_generations), best_fitness_per_generation)
        # plt.xlabel('Generation')
        # plt.ylabel('Best Fitness')
        # plt.title('Best Fitness per Generation')
        # plt.show()
        return self.decision