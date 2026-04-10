
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull
from matplotlib import patches
import matplotlib.gridspec as gridspec
from matplotlib.patches import Circle, Polygon
from matplotlib.lines import Line2D
from typing import List, Tuple
from std_msgs.msg import String
import rospy
from msgs.msg import AnimationSingleMeet, AnimationMeetList
from msgs.srv import (animation_status, animation_statusRequest, 
                      animation_meet, animation_meetRequest,
                    ) 
from concurrent.futures import ThreadPoolExecutor
from threading import Lock


EXP_STATUS_MARKER = {0:'o', 1:'v', 2:'D'}  # 0 for explore, 1 for travel, 2 for meet
EXP_STATUS_LINEWIDTH = {0:12, 1:12, 2:12}
EXP_STATUS_COLORS = {0:'blue', 1:'orange', 2:'green'}
INS_STATUS_MARKER = {0:'X', 1:'v', 2:'s'}  # 0 for inspect, 1 for travel, 2 for wait
INS_STATUS_LINEWIDTH = {0:12, 1:12, 2:12}
INS_STATUS_COLORS = {0: 'gold', 1: 'magenta', 2: 'cyan'}


TIMER_DURATION = 1
EXP_CORLOR = 'blue'
INS_CORLOR = [(1.0, 0.078, 0.576), (0.72, 0.33, 0.827), (1.0, 0.75, 0.75)]
FEATURE_CORLOR = [(1, 0, 0), (1, 0.55, 0), (1,1,0), (0.6, 1.0, 0.6)]

class ResultVisualizer:
    
    timer_duration: float = TIMER_DURATION

    def __init__(self, ifShowAnimation=0):

        # drone num
        self.exp_num = rospy.get_param("/exp_num")
        self.ins_num = rospy.get_param("/ins_num")
        
        # init data
        self.num_total_features: int = 0
        self.init_robot_status()
        self.init_feature_data()
        self.init_meet()
        
        # initialize the client to update the data
        self.ani_status_client = rospy.ServiceProxy("/exp_1/status_animation", animation_status)
        self.ins_ani_status_client = [rospy.ServiceProxy("/ins_"+str(i+1)+"/status_animation", animation_status) for i in range(self.ins_num)]
        
        self.ani_features_client = rospy.ServiceProxy("/featuers_stat", animation_status)
        self.ani_meet_client = rospy.ServiceProxy("/meet_animation", animation_meet)
         
        # visualization setup
        self.ifShowAnimation = ifShowAnimation
        self.plot_lock = Lock()
        self.save_fig_path = rospy.get_param("/save_fig_path")
        self.frame_id = 0
        self.ax_status, self.ax_meet, self.ax_feature, self.fig = self.init_vis()
        self.timer = rospy.Timer(rospy.Duration(TIMER_DURATION), self.plot_timer)
        self.plot_animation()

        
    def plot_timer(self, event):
        self.collect_data()
        with self.plot_lock:
            self.ifShowAnimation = 1
            

    def plot_animation(self):
        while not rospy.is_shutdown():
            while self.ifShowAnimation:
                self.update_figures()
                plt.pause(0.1)
                plt.savefig(self.save_fig_path + "frame_" + str(self.frame_id) + ".png")
                self.frame_id += TIMER_DURATION
                with self.plot_lock:
                    self.ifShowAnimation = 0
                # rospy.logerr("ifShowAnimation: %s", self.ifShowAnimation)
            
            
    # region initialization data functions
    
    def init_robot_status(self):
        rospy.logwarn("init_robot_status !!!!!!!!!!!!!!!!!!")
        self.exp_data = np.zeros((1, 2, 1))
        self.ins_data = np.ones((3, 1, 1))
       
            
    def init_feature_data(self):
        rospy.logwarn("init_feature_data !!!!!!!!!!!!!!!!!!")
        feature_msg = rospy.wait_for_message("bbox_feature_num", String)
        feature_data_str = feature_msg.data.strip()  # 去除字符串两端的空白字符
        index, value = feature_data_str.split(":")  # 分割字符串
        feature_num = int(value)  # 将数值部分转换为整数
        self.feature_data = np.zeros((4, 1, 1))
        self.feature_data[0, 0, 0] = feature_num 
        self.num_total_features = feature_num
        
        
    def init_meet(self):
        rospy.logwarn("init_meet !!!!!!!!!!!!!!!!!!")
        self.meet_data = []


    # endregion 
    
    
    # region plot functions including init and update
            
    def init_vis(self):

        fig = plt.figure(figsize=(45,10))
        gs = gridspec.GridSpec(15, 1)
        
        # 添加三个图
        ax_status = fig.add_subplot(gs[0:5,0], aspect='auto')
        ax_status.set_axis_off()
        for spine in ax_status.spines.values():
            spine.set_edgecolor('black')
            spine.set_linewidth(3)
        
        ax_meet = fig.add_subplot(gs[6:10,0], aspect='auto')
        ax_meet.set_axis_off()
        for spine in ax_meet.spines.values():
            spine.set_edgecolor('black')
            spine.set_linewidth(3)
        
        ax_feature = fig.add_subplot(gs[11:15,0], aspect='auto')
        ax_feature.set_axis_off()
        for spine in ax_feature.spines.values():
            spine.set_edgecolor('black')
            spine.set_linewidth(3)

        return ax_status, ax_meet, ax_feature, fig
    
    
    def update_figures(self):
        self.update_status()
        self.update_meet()
        self.update_features()
        

    def update_status(self):
        '''
            Args:
                exp_data: np.array, shape=(1, 1, num_steps) # 1 for explorer, 1 for status, num_steps for time
                ins_data: np.array, shape=(num_robots, 1, num_steps)  
        '''
        exp_data = self.exp_data
        ins_data = self.ins_data
        self.ax_status.clear()
        ax = self.ax_status

        ax.grid(False)
        if exp_data.shape[2] != ins_data.shape[2]:
            raise ValueError('The number of steps in the experiment data and the instance data must be the same.')
        num_steps = exp_data.shape[2]
        num_ins = ins_data.shape[0]

        # create the plot for each ins
        for i in range(num_ins):
            start = 0
            while start < num_steps:
                end = start
                while end < num_steps and ins_data[i, 0, end] == ins_data[i, 0, start]:
                    end += 1
                ax.plot([start*TIMER_DURATION, end*TIMER_DURATION], [i, i], color=INS_STATUS_COLORS[ins_data[i, 0, start]], linewidth=INS_STATUS_LINEWIDTH[ins_data[i, 0, start]], zorder=1)
                start = end  # Move to the next segment
                
        # create the plot for exp
        start = 0
        while start < num_steps:
            end = start
            while end < num_steps and exp_data[0, 0, end] == exp_data[0, 0, start]:
                end += 1
            if exp_data[0, 0, start] == 2:
                pass
            else:
                ax.plot([start*TIMER_DURATION, end*TIMER_DURATION], 
                        [num_ins, num_ins], 
                        color=EXP_STATUS_COLORS[exp_data[0, 0, start]], 
                        linewidth=EXP_STATUS_LINEWIDTH[exp_data[0, 0, start]], zorder=1)
            start = end
            
        # add the markers for the meet status of exp
        for i in range(exp_data.shape[2]):
            if exp_data[0, 1, i] >0:
                ax.plot(exp_data[0, 1, i], num_ins, color=EXP_STATUS_COLORS[2], marker='o', markersize=8, zorder=2)

                            
        ytickes = ['ins_1', 'ins_2', 'ins_3', 'exp_1']        
        ax.set_yticks(range(num_ins + 1))
        ax.set_yticklabels(ytickes, fontsize=10, fontweight='bold')
        
        # ax.set_title('Status of Explorer and Inspectors', fontsize=12, fontweight='bold', ha='center', va='center', transform=ax.transAxes)
        # ax.set_xticklabels([x * TIMER_DURATION for x in range(0, num_steps, 1)], fontsize=10, fontweight='bold', color='black')
        ax.set_xlim(0, num_steps+1)
        ax.set_ylim(-1, num_ins+1)
        
        xtickes = ax.get_xticklabels()
        for label in xtickes:
            label.set_fontsize(10)
            label.set_fontweight('bold')
            
        # ax.text(1.03, -0.05, '/s', transform=ax.transAxes, ha='right', va='top', fontsize=12, fontweight='bold', color='black')
        

        # create a legend
        legend_elements = [
        Line2D([0], [0], color=EXP_STATUS_COLORS[0], lw=EXP_STATUS_LINEWIDTH[0], label='explore'),
        Line2D([0], [0], color=INS_STATUS_COLORS[0], lw=INS_STATUS_LINEWIDTH[0], label='inspect'),
        Line2D([0], [0], color=EXP_STATUS_COLORS[1], lw=EXP_STATUS_LINEWIDTH[1], label='travel'),
        Line2D([0], [0], color=INS_STATUS_COLORS[1], lw=INS_STATUS_LINEWIDTH[1], label='travel'),
        Line2D([0], [0], color=EXP_STATUS_COLORS[2], lw=EXP_STATUS_LINEWIDTH[2], label='meet', marker='o', linestyle='None'),
        Line2D([0], [0], color=INS_STATUS_COLORS[2], lw=INS_STATUS_LINEWIDTH[2], label='wait')
            ]
        
        
        # ax.legend(handles=legend_elements, loc='upper left', ncol=3, framealpha=0.5)

        
    def update_meet(self):
        '''
            Args:
                data: List[Tuple[ins_id, feature_num, meeting_time]]
        '''
        num_ins = self.ins_num
        self.ax_meet.clear()
        ax = self.ax_meet
        ax.grid(False)
        max_meeting_time: float = 0
        total_allocated_feature_count = 0
        total_meeting_count = 0

        for data in self.meet_data:
            if len(data) == 0:
                continue
            data: List[Tuple[str, int, float]]

            # here deal with no meeting but decision making
            if data[0][0] == '0':
                decision_making_time = data[0][2]-1
                largest_meeting_time = data[0][2]
            else:  
                decision_making_time = data[0][2]-1 # TODO: change the decision_making_time to the real decision_making_time
                largest_meeting_time: float = data[-1][2]
            if largest_meeting_time > max_meeting_time:
                max_meeting_time = largest_meeting_time

            ax.add_patch(patches.Rectangle((decision_making_time, -1), largest_meeting_time-decision_making_time+1, len(data)+4, facecolor='gray', linewidth=2, alpha=0.5))
            for ins_id, feature_num, meeting_time in data:
                if ins_id == '0':
                    continue
                id = int(ins_id)
                
                vertical_line = Line2D([meeting_time, meeting_time], [id - 1 - 0.3, id - 1 + 0.3], color='black', linewidth=6)
                ax.add_line(vertical_line)
                ax.plot(meeting_time, id - 1 + 0.3, marker='o', markersize=14, color='yellow', markeredgecolor='black')
                ax.plot(meeting_time, id - 1 - 0.3, marker='o', markersize=14, color='yellow', markeredgecolor='black')
                
                ax.text(meeting_time+0.18, id-1+0.7, str(feature_num), fontsize=20, ha='center', va='center', color='black', fontweight='bold')
                total_allocated_feature_count += feature_num
                total_meeting_count += 1
                    
        # ax.set_title('Decision Result', fontsize=12, fontweight='bold', ha='center', va='center', transform=ax.transAxes)
        # 加上坐标轴
        ax.set_xlim(0, max_meeting_time+10)
        # ax.set_xticklabels([x * TIMER_DURATION for x in range(0, max_meeting_time+10, 1)], fontsize=10, fontweight='bold', color='black')
        ax.set_ylim(-1, num_ins)
        ytickes = ['ins_1', 'ins_2', 'ins_3']        
        ax.set_yticks(range(num_ins))
        ax.set_yticklabels(ytickes, fontsize=10, fontweight='bold')
        # 绘制legend，类型为矩形
        legend_elements = [
            Line2D([0, 0], [0, -0.2], color='black', linewidth=2, label='Feature', marker='o',  markersize=6, markerfacecolor='yellow', markeredgecolor='black'),  # 竖直线段
            Line2D([0], [0], color='lightgray', alpha=0.5, linewidth=15, label='Decision Result')  # 原灰色矩形
        ]
        # ax.legend(handles=legend_elements, loc='upper left', fontsize=12)
        for i in range(self.ins_num):
            ax.axhline(y=i, color=INS_CORLOR[i], linestyle='-', linewidth=6)
        xtickes = ax.get_xticklabels()
        for label in xtickes:
            label.set_fontsize(10)
            label.set_fontweight('bold')
            
        # ax.text(1.03, -0.05, '/s', transform=ax.transAxes, ha='right', va='top', fontsize=12, fontweight='bold', color='black')

        from colorama import Fore, Style
        
        if total_meeting_count != 0:
            print(f"{Fore.GREEN}Total meeting count: {round(total_allocated_feature_count/total_meeting_count,2)}{Style.RESET_ALL}")
        
        
    def update_features(self):
        '''
            Args:
                feature_data: np.array, shape=(num_type_features, num, num_steps)        
        '''
        feature_data = self.feature_data
        self.ax_feature.clear()
        ax = self.ax_feature
        ax.grid(False)
        num_steps = feature_data.shape[2]
        num_features = feature_data.shape[0]
        feature_lable = ['registered', 'found', 'allocated', 'inspected']
        # Create the plot for each feature
        for i in range(num_features):
            ax.plot([i * TIMER_DURATION for i in range(num_steps)], feature_data[i, 0, :], color=FEATURE_CORLOR[i], linewidth=8, label=feature_lable[i])
            
        # ax.set_title('Meeting Features', fontsize=12, fontweight='bold', ha='center', va='center', transform=ax.transAxes)
        ax.set_xlim(0, num_steps+1)
        # ax.set_xticklabels([x * TIMER_DURATION for x in range(0, num_steps, 1)], fontsize=10, fontweight='bold', color='black')
        ax.set_ylim(-10, self.num_total_features+15)
        ax.axhline(y=self.num_total_features, color='black', linestyle='--', linewidth=6)
        # ax.legend(loc='center left', bbox_to_anchor=(0, 0.5), framealpha=0.5)

        
        xtickes = ax.get_xticklabels()
        for label in xtickes:
            label.set_fontsize(10)
            label.set_fontweight('bold')
            
        ytickes = ax.get_yticklabels()
        for label in ytickes:
            label.set_fontsize(10)
            label.set_fontweight('bold')
        
        # ax.text(1.03, -0.05, '/s', transform=ax.transAxes, ha='right', va='top', fontsize=12, fontweight='bold', color='black')

    
    # endregion
    
    
    # region collect functions to update data
    
    def collect_data(self):
        self.collect_status_data()
        self.collect_feature_data()
        self.collect_meet_data()
        
    
    def call_service(self, robot_id: str):
        try:
            req = animation_statusRequest()
            req.robot_id = robot_id
            if robot_id == 'exp_1':
                ani_status_client = self.ani_status_client
            else:
                ins_id = int(robot_id.split("_")[1])
                ani_status_client = self.ins_ani_status_client[ins_id-1]
            ani_status_client.wait_for_service()
            status_data = ani_status_client.call(req)
            robot_id, status, time_info = status_data.status.split(",")
  
            return int(status), time_info
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s"%e)
    
    
    def collect_status_data(self):
        '''
            simultaneously get the status data of four robots
        '''
        # ============ part1 collect data ============
        with ThreadPoolExecutor(max_workers=4) as executor:
            future_exp_1 = executor.submit(self.call_service, 'exp_1')
            future_ins_1 = executor.submit(self.call_service, 'ins_1')
            future_ins_2 = executor.submit(self.call_service, 'ins_2')
            future_ins_3 = executor.submit(self.call_service, 'ins_3')
                
            exp_1_status, exp_1_time_info = future_exp_1.result()
            ins_1_status, _ = future_ins_1.result()
            ins_2_status, _ = future_ins_2.result()
            ins_3_status, _ = future_ins_3.result()
            
            # rospy.loginfo("Responses: exp_1: %s, ins_1: %s, ins_2: %s, ins_3: %s",
            #           exp_1_status, ins_1_status, ins_2_status, ins_3_status)
        
        # ============ part2 update data ============
        exp_1_time = float(exp_1_time_info) if exp_1_time_info !='None' else 0
        exp_new_data  = np.array([exp_1_status, exp_1_time]).reshape(1, 2, 1)
        self.exp_data = np.concatenate((self.exp_data, exp_new_data), axis=2)
        
        ins_new_data = np.array([[ins_1_status], [ins_2_status], [ins_3_status]]).reshape(3, 1, 1)
        ins_new_data = np.array([[ins_1_status], [ins_2_status], [ins_3_status]]).reshape(3, 1, 1)
        self.ins_data = np.concatenate((self.ins_data, ins_new_data), axis=2)
        
    
    def collect_feature_data(self):
        # ========== part1 collect data ============
        try:
            req = animation_statusRequest()
            req.robot_id = 'feature'
            self.ani_features_client.wait_for_service()
            feature_data = self.ani_features_client.call(req)
            feature = feature_data.status
            reg, found, alloc, insp = feature.split(",")
            # rospy.logwarn(f"reg: {reg}, found: {found}, alloc: {alloc}, insp: {insp}")
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s"%e)

        # ========== part2 update data ============
        feature_data = np.array([int(reg), int(found), int(alloc), int(insp)]).reshape(4, 1, 1)
        self.feature_data = np.append(self.feature_data, feature_data, axis=2)
    
    
    def collect_meet_data(self):
        # ============ part1 collect data ============
        try:
            req = animation_meetRequest()
            req.exp_id = 'exp_1'
            self.ani_meet_client.wait_for_service()
            meet_data = self.ani_meet_client.call(req)
            if meet_data.meet_list.meet_list == []:
                ani_meet_data_list = []
            else:
                animation_meet_data_list = meet_data.meet_list.meet_list
                ani_meet_data_list = []
                for data in animation_meet_data_list:
                    ins_id, feature_num, meeting_time = data.ins_id, data.feature_num, data.meet_time
                    ani_meet_data_list.append((ins_id, feature_num, meeting_time))
                    
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s"%e)
            
        # ============ part2 update data ============
        if ani_meet_data_list == []:
            pass
        else:
            self.meet_data.append(ani_meet_data_list)
            rospy.logerr("meet_data: %s", ani_meet_data_list)

    # endregion
        