import os
import sys
import json
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QSizePolicy, QMenu, QLabel, QWidgetAction, QSpacerItem, QTableWidget, QTableWidgetItem, QScrollArea, QPushButton
from PyQt5.QtChart import QChartView, QChart, QPieSeries, QPieSlice, QBarSet, QBarSeries, QBarCategoryAxis, QValueAxis
from PyQt5.QtGui import QColor, QPainter, QFont, QPixmap, QBrush, QImage, QPen
from PyQt5.QtCore import Qt, QEvent, QTimer, QMargins, QTime, pyqtSignal, QThread
from pyqtgraph import PlotWidget, mkPen, BarGraphItem
from qtwidgets import AnimatedToggle
from random import randint
from PyQt5 import uic
import pyqtgraph
import mysql.connector
import requests
import socket
import webbrowser
from datetime import datetime
import resources  # needed for loading resourceskkk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import cv_bridge

current_dir = os.path.dirname(os.path.abspath(__file__)) # 현재 스크립트의 디렉토리를 가져오고, 프로젝트 루트로 이동하는 상대 경로를 추가
relative_path = os.path.join(current_dir, '../')  # 상위 폴더로 이동
sys.path.append(relative_path)

from gui_import_class.video_subscriber import ImageSubscriber
from gui_import_class.video_subscriber import amclSubscriber
from gui_import_class.topic_publisher_node import GuiTopicPublisher
# print(sys.path)
class MessageReceiver(QThread):
    received = pyqtSignal(bytes)

    def __init__(self, sock):
        super().__init__()
        self.sock = sock
        self.running = True

    def run(self):
        try:
            while self.running:
                header = b''
                while len(header) < 2:
                    header += self.sock.recv(2 - len(header))

                if header == b'RS':
                    data = self.sock.recv(2)
                    print(f"receive raw data: {data}")
                    datan= int.from_bytes(data[:1], byteorder="big")
                    print(f"real data : {datan}")

                    self.received.emit(data)
        except Exception as e:
            print(f"메시지 수신 오류: {e}")

class OrchardGUI(QMainWindow):
    def __init__(self,video_node=None,amcl_node=None, publish_node=None):
        self.connection_to_database = False
        self.video_node = video_node
        self.amcl_node = amcl_node
        self.publish_node = publish_node
        self.weather_internet = True # 인터넷 불가 대비용 플래그
        super(OrchardGUI, self).__init__()
        # Load the sidebar UI
        # uic.loadUi("/home/ksm/ws/MLDL/projects/group_pollination/src/gui_test/sidebar.ui", self)
        # uic.loadUi("GUI/sidebar.ui", self)
        # uic.loadUi("src/GUI/sidebar.ui", self)
        uic.loadUi(os.path.join(relative_path,"applecare_gui_pkg/sidebar.ui"),self)
        self.setWindowTitle("Apple Care")
        self.icon_name_widget.setHidden(True) # Hide an element (icon_name_widget)

        # Connect buttons to corresponding pages
        self.home_btn_1.clicked.connect(self.switch_to_home)
        self.home_btn_2.clicked.connect(self.switch_to_home)
        self.map_btn_1.clicked.connect(self.switch_to_map)
        self.map_btn_2.clicked.connect(self.switch_to_map)
        self.stat_btn_1.clicked.connect(self.switch_to_stat)
        self.stat_btn_2.clicked.connect(self.switch_to_stat)
        self.timeset_btn_1.clicked.connect(self.switch_to_settings)
        self.timeset_btn_2.clicked.connect(self.switch_to_settings)

        # -------------------------------------------------------------------------
        # main dashboard page 1
        self.add_pie_chart()
        self.create_dynamic_plot()

        self.api_key = '727250f5a30e68d3bc3896624421ddab'
        self.get_weather_data()

        self.wholesaler_price_btn.clicked.connect(self.open_webbrowser)
        self.auction_status_btn.clicked.connect(self.open_webbrowser)
        self.online_market_btn.clicked.connect(self.open_webbrowser)

        self.robot_devices()
        self.robotControl_btn.clicked.connect(self.switch_to_map)

        # -------------------------------------------------------------------------
        # 2d map and robot control page 2
        
        # Initialize pollination data
        self.pollination = 30
        self.total = 50
        self.percentage = round((self.pollination / self.total) * 100)

        # Setup hover menu for tree icons and update labels periodically
        self.tree_labels()

        self.tree1_icon.installEventFilter(self)
        self.tree2_icon.installEventFilter(self)

        self.pollination_rate()  # Call the method to update the UI
        self.scan_rate()
        
        # -------------------------------------------------------------------------
        # statistics and analytics page 3

        self.pollinated_tree_count()
        self.monthly_progress_chart()
        self.display_log()

        # set up TCP socket
        # self.server_ip = "192.168.0.134"
        # self.server_port = 1235

        # self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.sock.connect((self.server_ip, self.server_port))  

        # self.scanstart_btn.clicked.connect(self.send_robotControl)
        
        # self.receiver_thread = MessageReceiver(self.sock)
        # self.receiver_thread.received.connect(self.process_received_message)
        # self.receiver_thread.start()

        # -------------------------------------------------------------------------
        # set schedule settings page 4
        self.onoff_schedule()
        # -------------------------------------------------------------------------
        # 타이머 관리
        self.init_timers(self.get_weather_data, 300000) # 날씨 타이머
        self.init_timers(self.update_dynamic_plot, 1000) # 다이나믹 플롯 타이머
        self.init_timers(self.add_pie_chart, 86400000) #파이차트 타이머
        self.init_timers(self.tree_labels, 10000) # 트리 라벨 타이머
        self.home_btn_1.click() # 시작하자마자 홈버튼 클릭해서 흰색 활성화
        self.scanstart_btn.clicked.connect(self.send_scanControl)
        self.pollistart_btn.clicked.connect(self.send_polliControl)

        self.video_start.clicked.connect(self.start_video)
        self.video_flag = False

        self.pixmap = QPixmap()
        self.video_timer = QTimer(self)
        self.video_timer.timeout.connect(self.update_video)
        #-------------------------------------------------------------------------------

        # -------------------------------------------------------------------------------
        # 테스트 관리용 위젯
        self.test_page_button.clicked.connect(self.switch_to_test)

        self.map_pixmap = QPixmap('/home/zeki/my_mobile/src/my_robot_navigation/maps/map_name.pgm')
        self.test_map_label.setPixmap(self.map_pixmap)
        self.amcl_timer = QTimer(self)
        self.amcl_timer.timeout.connect(self.update_amcl)
        self.amcl_timer.start(100)
        
        # 홈 화면 모니봇 상태 표시용 라벨/ 이미지로 대체해야함
        self.home_monibot_state_label.setText("normal")


        # ------------------------------------------------------------
        # test log page
        self.test_log_btn.clicked.connect(self.switch_to_test_log)
        from PyQt5.QtWidgets import QHeaderView
        # self.area_log_show_box.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.area_log_show_box.setColumnWidth(0, 100)  # 첫 번째 열의 너비를 100으로 설정
        self.area_log_show_box.setColumnWidth(1, 150)  # 두 번째 열의 너비를 150으로 설정
        self.area_log_show_box.setColumnWidth(2, 150) 
        self.area_log_show_box.setColumnWidth(3, 209) 
        
        self.search_log_btn.clicked.connect(self.search_area_log)

        self.task_upload_btn.clicked.connect(self.upload_task)
        
    def upload_task(self):
        task_type = self.reserve_type_combo.currentText()
        if task_type == 'scan':
            task_type = 0
        else:
            task_type = 1


        try:
            A_sequence = int(self.A_sequence.text())
        except:
            A_sequence = None

        try:
            B_sequence = int(self.B_sequence.text())
        except:
            B_sequence = None
        print("#########################################")
        print(self.send_now.isChecked())
        print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        if self.send_now.isChecked():
            instant_task= True
            year = None
            month = None
            day = None
            hour = None
            minute = None
        else:
            instant_task = False
            fulldate = self.reserve_date.date()
            year = fulldate.year()
            month = fulldate.month()
            day = fulldate.day()

            time = self.reserve_time.time()
            hour = time.hour()
            minute = time.minute()
        
        self.publish_node.publish_topic(topic_name="task_topic",task_type=task_type, 
                                        area=None,priority_area=None,
                                        instant_task = instant_task,
                                        A_sequence=A_sequence,
                                        B_sequence=B_sequence,
                                        year=year,month=month,day=day,
                                        hour=hour, minute=minute) # task_type 0: scan, 1: pollinate
        
        
    def update_amcl(self):
        amcl= self.amcl_node.get_pose()
        if amcl is not None:
            x= int(amcl.pose.pose.position.x)
            y= int(amcl.pose.pose.position.y)

            painter = QPainter(self.map_pixmap)
            pen = QPen(Qt.red, 3)  # 빨간색 펜, 두께 3
            painter.setPen(pen)
            painter.drawRect(x, y, 100, 100)
            painter.end()
            self.test_map_label.setPixmap(self.map_pixmap)
            
    def switch_to_test(self):
        self.stackedWidget.setCurrentIndex(4)

    def switch_to_test_log(self):
        self.stackedWidget.setCurrentIndex(5)

    def search_area_log(self):
        self.area_log_show_box.clear()
        area=self.area_check_box.currentText()

        start_date=self.date_box.text()
        disease=self.disease_check_box.currentText()

        search_list=[area,disease]
        for idx,key in enumerate(search_list):
            if key == "All":
                search_list[idx] = ""

        sql = """SELECT * FROM celeb
                WHERE birthday BETWEEN %s AND %s 
                AND sex LIKE %s 
                AND job_title LIKE %s 
                AND agency LIKE %s;
            """
        self.send_to_db_manager(sql)
        # 이런 식으로 result 를 받아서 출력 시켜야 함

        # result = cur.fetchall()
        # for line in result:
        #     # print(line)

        #     row = self.tableWidget.rowCount()
        #     self.tableWidget.insertRow(row)
        #     self.tableWidget.setItem(row,0,QTableWidgetItem(line[0]))#ID############3
        #     self.tableWidget.setItem(row,1,QTableWidgetItem(line[1]))#NAME
        #     self.tableWidget.setItem(row,2,QTableWidgetItem(line[2].strftime("%Y-%m-%d")))#BIRTHDAY
        #     self.tableWidget.setItem(row,3,QTableWidgetItem(line[3]))#AGE
        #     self.tableWidget.setItem(row,4,QTableWidgetItem(line[4]))#SEX
        #     self.tableWidget.setItem(row,5,QTableWidgetItem(line[5]))#JOB_TITLE
        #     self.tableWidget.setItem(row,6,QTableWidgetItem(line[6]))#AGENCY
    
    def send_to_db_manager(self,sql):
        # need to include DB manager node, or publish sql topic to db manager
        pass

        
    def start_video(self):
        if self.video_flag == False:
            self.video_timer.start(100)
            self.video_flag = not self.video_flag
        elif self.video_flag == True:
            self.video_timer.stop()
            self.video_flag = not self.video_flag
            self.video_label.clear()
            


    def update_video(self):
        frame = self.video_node.get_frame()
        if frame is not None:
            # Convert the frame to a QImage and display it on the QLabel
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame_rgb.shape
            bytes_per_line = ch * w
            qimage = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(qimage))

            qimage = QImage(frame_rgb,w,h,w*ch, QImage.Format_RGB888)
            self.pixmap = self.pixmap.fromImage(qimage)
            self.pixmap = self.pixmap.scaled(self.video_label.width(),self.video_label.height())

            self.video_label.setPixmap(self.pixmap)
    #     pass

    def switch_to_home(self):
        self.stackedWidget.setCurrentIndex(0)

    def switch_to_map(self):
        self.stackedWidget.setCurrentIndex(1)

    def switch_to_stat(self):
        self.stackedWidget.setCurrentIndex(2)

    def switch_to_settings(self):
        self.stackedWidget.setCurrentIndex(3)

    def init_timers(self, function, interval):
        # Create a single QTimer for updating multiple components
        self.timer = QTimer(self)
        self.timer.timeout.connect(function)
        self.timer.start(interval) 


    # -------------------------------------------------------------------------------------------------------------------------------
    # main dashboard (page 1)
    # -------------------------------------------------------------------------------------------------------------------------------

    def robot_devices(self):
        pollibot_toggle = AnimatedToggle(
                checked_color="#ffbb00",
                pulse_checked_color="#44FFB000"
            )
        monibot_toggle = AnimatedToggle(
                checked_color="#ffbb00",
                pulse_checked_color="#44FFB000"
            )

        pollibot_layout = QVBoxLayout(self.pollibot_toggle)  # Assuming pollibot_toggle is a QLabel
        pollibot_layout.addWidget(pollibot_toggle)
        pollibot_layout.setAlignment(Qt.AlignCenter)  # Align the toggle button to the center

        monibot_layout = QVBoxLayout(self.monibot_toggle)  # Assuming monibot_toggle is a QLabel
        monibot_layout.addWidget(monibot_toggle)
        monibot_layout.setAlignment(Qt.AlignCenter)  # Align the toggle button to the center

        pollibot_layout.setContentsMargins(0, 0, 0, 0)
        monibot_layout.setContentsMargins(0, 0, 0, 0)

    
    def create_dynamic_plot(self):
        self.plot_graph = PlotWidget()
        self.plot_graph.setBackgroundBrush(QColor("#ede3d6"))

        # Set line pen: blue color and thickness of 2
        pen = mkPen(color='#8b8075', width=2)

        # Set title and labels with customized styles
        #self.plot_graph.setTitle("실시간 착과량", font="NanumSquareRound ExtraBold", color="black", size="13pt")
        styles = {"color": "black", "font-size": "11px"}
        self.plot_graph.setLabel("left", "총 착과량", **styles)
        self.plot_graph.setLabel("bottom", "일별", **styles)
        self.plot_graph.addLegend()
        self.plot_graph.showGrid(x=True, y=True)
        self.plot_graph.setYRange(0, 40)

        # Sample data for plotting
        self.time = list(range(10))
        self.count = [randint(20, 40) for _ in range(10)]
        
        # Create the plot with customized line and symbol colors
        self.line = self.plot_graph.plot(
            self.time, self.count,
            pen=pen,  # Blue line with thickness 2
            symbol='o',  # Use circle symbol
            symbolSize=6,  # Symbol size
            symbolBrush=QColor("#6e6053"),  # Fill color for symbols
            symbolPen=mkPen("#6e6053")  # Outline color for symbols
        )

        # Set layout for the groupbox
        layout = QVBoxLayout()
        layout.addWidget(self.plot_graph)
        self.linechart_area.setLayout(layout)

        # self.init_timers(self.update_dynamic_plot, 1000)


    def update_dynamic_plot(self):
        self.time = self.time[1:]
        self.time.append(self.time[-1] + 1)

        self.count = self.count[1:]
        self.count.append(randint(20, 40))

        self.line.setData(self.time, self.count)

    def add_pie_chart(self):
        datas = [
            ("달성", 40, "#544b40", "#746858"),
            ("미달성", 40, "#6e6053", "#6a5030"),
            ("예외", 20, "#766c5f", "#9c8e7c"),
        ]

        chart = QChart()
        chart.setBackgroundBrush(QColor("#ede3d6"))
        # chart.setTitle("목표 달성률")
        # title_font = QFont("NanumSquareRound", 14, QFont.Bold)
        # chart.setTitleFont(title_font)
        #chart.setTitleMargins(0, 0, 0, 20)  # Increase bottom margin of the title
        chart.setMargins(QMargins(0, 0, 0, 0)) # Adjust margins around the chart (top, left, right, bottom)

        chart.legend().hide()
        chart.setAnimationOptions(QChart.SeriesAnimations)

        outer_series = QPieSeries()
        outer_series.setHoleSize(0.35)

        for name, value, primary_color, _ in datas:
            outer_slice = QPieSlice(name, value)
            outer_slice.setLabelVisible()
            outer_slice.setColor(QColor(primary_color))
            outer_slice.setLabelBrush(QColor(primary_color))
            outer_series.append(outer_slice)

        for slice_ in outer_series.slices():
            if slice_.label() == "달성":
                slice_.setExploded(True)
                slice_.setExplodeDistanceFactor(0.25)
                slice_.setLabelVisible(True)

        chart.addSeries(outer_series)

        for slice_ in outer_series.slices():
            color = 'black' if slice_.percentage() > 0.1 else 'white'
            # label = f"<p font='NanumSquareRound ExtraBold' align='left' style='color:{color}'>{slice_.label()}<br>{round(slice_.percentage() * 100)}%</p>"
            label = f"<p style='font-family:NanumSquareRound ExtraBold; font-size:8pt; color:{color};'>{slice_.label()}<br>{round(slice_.percentage() * 100)}%</p>"
            slice_.setLabelPosition(QPieSlice.LabelOutside)
            slice_.setLabel(label)

        chart_view = QChartView(chart)
        chart_view.setRenderHint(QPainter.Antialiasing)
        chart_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        chart_view.setMinimumSize(280, 200)  # Set a bigger minimum size for the chart

        layout = QVBoxLayout()
        layout.addItem(QSpacerItem(0, 0))  # Add a vertical spacer (40px) above the chart
        layout.addWidget(chart_view, alignment=Qt.AlignCenter)  # Align the chart to the center
        self.piechart_groupbox.setLayout(layout)

        # self.init_timers(self.add_pie_chart, 86400000)

    def get_weather_data(self):
        if self.weather_internet == False:
            return
        try:
            city = "Seoul"
            url = f"http://api.openweathermap.org/data/2.5/weather?q={city}&appid={self.api_key}&units=metric"
            response = requests.get(url)
            data = response.json()
            print(response.status_code)
            if response.status_code == 200:
                icon_id = data['weather'][0]['icon']
                temperature = data['main']['temp']
                wind_speed = data['wind']['speed']
                humidity = data['main']['humidity']
                precipitation = 0
                if 'rain' in data:
                    precipitation = data['rain'].get('1h', 0)  # Precipitation in last 1 hour, in mm
                elif 'snow' in data:
                    precipitation = data['snow'].get('1h', 0)  # Snow in last 1 hour, in mm

                current_year = datetime.now().strftime("%Y")
                current_month = datetime.now().strftime("%m")
                current_date = datetime.now().strftime("%d")
                current_hour = datetime.now().strftime("%H")
                current_minute = datetime.now().strftime("%M")
                
                weather_text = f"""
                    <p style="font-size: 13px; text-align: left; margin: 2.5px; ">위치: 서울시 </p>
                    <p style="font-size: 13px; text-align: left; margin: 2.5px; ">온도: {temperature} °C </p>
                    <p style="font-size: 13px; text-align: left; margin: 2.5px; ">습도: {humidity} % </p>
                    <p style="font-size: 13px; text-align: left; margin: 2.5px; ">강수량: {precipitation} mm </p>
                    <p style="font-size: 13px; text-align: left; margin: 1.5px; ">풍속: {wind_speed} km/h</p>
                    """
                
                icon_url = f"http://openweathermap.org/img/wn/{icon_id}@2x.png"
                icon_response = requests.get(icon_url)
                pixmap = QPixmap()
                pixmap.loadFromData(icon_response.content)

                self.weather_icon = QLabel()
                self.weather_icon.setPixmap(pixmap)
                self.weather_icon.setAlignment(Qt.AlignRight)

                self.weather_label.setText(weather_text)
                self.weather_label.setFont(QFont("NanumSquareRound ExtraBold"))

            else:
                self.weather_label.setText("Error fetching weather data.")
                current_year,current_month,current_date,current_hour,current_minute = "-","-","-","-","-"
        except Exception as e:
            self.weather_label.setText(f"Failed to fetch weather data: {str(e)}")
            current_year,current_month,current_date,current_hour,current_minute = "-","-","-","-","-"
            self.weather_internet = False
            print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5")
        
        # self.init_timers(self.get_weather_data, 30000)

        print("updating weather")

        self.datetime.setText(f"""
                <p style="font-size: 14px; text-align: center; font-weight: bold; margin: 2px; ">
                {current_year}년 {current_month}월 {current_date}일    {current_hour}:{current_minute}</p>""")
        
        # 30초에 한번씩 날씨 업데이트 받게 되면 이미 존재하는 layout에 덮어씌운다고 경고문이 뜸으로 예외처리 필요.
        if self.text_widget.layout() is not None:
            text_layout = self.text_widget.layout()
            while text_layout.count():
                item = text_layout.takeAt(0)
                widget = item.widget()
                # if widget is not None:
                #     widget.deleteLater()

        if self.text_widget.layout() is None:
            text_layout = QVBoxLayout()
            text_layout.addWidget(self.weather_label)
            self.text_widget.setLayout(text_layout)

        if self.icon_widget.layout() is not None:
            icon_layout = self.icon_widget.layout()
            while icon_layout.count():
                item = icon_layout.takeAt(0)
                widget = item.widget()
                # if widget is not None:
                #     widget.deleteLater()

        if self.icon_widget.layout() is None:
            icon_layout = QVBoxLayout()
            icon_layout.addWidget(self.weather_icon)
            self.icon_widget.setLayout(icon_layout)


    def open_webbrowser(self):
        button = self.sender()  # Get the button that was clicked
        button.setStyleSheet("""
            QPushButton {
                color: rgb(255, 255, 255);
                background-color: rgb(167, 153, 139);
            }
            QPushButton:hover {
                color: rgb(255, 255, 255);
                background-color: rgb(148, 135, 123);
            }
        """)
        if button == self.wholesaler_price_btn:
            webbrowser.open('https://www.kamis.or.kr/customer/price/wholesale/item.do')
        elif button == self.auction_status_btn:
            webbrowser.open('https://at.agromarket.kr/index.do;jsessionid=E3F09F3582F1AF390948AED39E335E6B')
        elif button == self.online_market_btn:
            webbrowser.open('https://kafb2b.or.kr/client/mn/main/main.do')


    # -------------------------------------------------------------------------------------------------------------------------------
    # 2d map and robot control page 2
    # -------------------------------------------------------------------------------------------------------------------------------

    def tree_labels(self):
        # update tree labels
        self.menu = QMenu(self)
        self.menu_label = QLabel()
        self.menu_label.setStyleSheet('padding: 5px; font-size: 14px;')
        self.menu_label.setWordWrap(True)
        widget_action = QWidgetAction(self.menu)
        widget_action.setDefaultWidget(self.menu_label)
        self.menu.addAction(widget_action)

        labels = [self.tree1_label, self.tree2_label]
        for label in labels:
            label.setFont(QFont("NanumSquareRound ExtraBold", 20))
            if self.percentage >= 75:
                label.setText(f"{round(self.percentage)}%")
                label.setStyleSheet("background-color: #8fb84a;")
            elif self.percentage >= 50:
                label.setText(f"{round(self.percentage)}%")
                label.setStyleSheet("background-color: #f8cf3f;")
            else:
                label.setText(f"{round(self.percentage)}%")
                label.setStyleSheet("background-color: #cb4343;")
        
        # self.init_timers(self.tree_labels, 10000)


    def eventFilter(self, source, event):
        combined_results = self.retrieve_from_database()  # Get the tree data from the database

        if event.type() == QEvent.Enter:
            if source == self.tree1_icon:
                tree_no = 1
            elif source == self.tree2_icon:
                tree_no = 2
            else:
                return super().eventFilter(source, event)

            # Find the correct data for the hovered tree
            tree_data = next((tree for tree in combined_results if tree[0] == tree_no), None)

            if tree_data:
                tree_id = tree_data[0]
                x_axis = tree_data[1]
                y_axis = tree_data[2]
                flower_count = tree_data[3]
                bud_count = tree_data[4]
                pollination_count = tree_data[5]
                season = tree_data[6]
                plant_date = tree_data[7]
                update_date = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    # 나무 위치 없어도 됨, 
                label_text = f"""
                    <p style="font-size: 17px; font-weight: bold;">사과나무 정보</p>
                    <p>꽃 개화 상태: {flower_count}/{flower_count + bud_count + pollination_count}</p>
                    <p>나무 식별번호: {tree_id}</p>
                    <p>나무 위치: ({x_axis}, {y_axis})</p>
                    <p>심은 일자: {plant_date}</p>
                    <p>시즌: {season}</p>
                    <p style="font-size: 9px; text-align: right;">마지막 업데이트: {update_date}</p>
                """

                self.menu_label.setText(label_text)
                self.menu_label.setFont(QFont("NanumSquareRound ExtraBold"))
                self.menu.exec_(source.mapToGlobal(source.rect().bottomLeft()))

        elif event.type() == QEvent.Leave:
            self.menu.hide()

        return super().eventFilter(source, event)

    
    def pollination_rate(self):
        self.pollination += 5  # Update this as needed
        bars = [self.polli_progressbar, self.polliprogressbar]
        rate = round((self.pollination/self.total) * 100)
        for bar in bars:
            bar.setValue(rate)
            if rate >= 60:
                bar.setStyleSheet("""
                    QProgressBar {
                        border: 1px solid grey;
                        border-radius: 2px;
                        text-align: center;
                    }
                    QProgressBar::chunk {
                        background-color: #8fb84a;
                        border-radius: 2px;
                    }
                """)
            else:
                bar.setStyleSheet("""
                    QProgressBar {
                        border: 1px solid grey;
                        border-radius: 2px;
                        text-align: center;
                    }
                    QProgressBar::chunk {
                        background-color: #f3ba5c;
                        border-radius: 2px;
                    }
                """)
                
    def scan_rate(self):
        bars = [self.scan_progressbar, self.scanprogressBar]
        rate = round((30/100) * 100)
        for bar in bars:
            bar.setValue(rate)
            if rate >= 60:
                bar.setStyleSheet("""
                    QProgressBar {
                        border: 1px solid grey;
                        border-radius: 2px;
                        text-align: center;
                    }
                    QProgressBar::chunk {
                        background-color: #8fb84a;
                        border-radius: 2px;
                    }
                """)
            else:
                bar.setStyleSheet("""
                    QProgressBar {
                        border: 1px solid grey;
                        border-radius: 2px;
                        text-align: center;
                    }
                    QProgressBar::chunk {
                        background-color: #f3ba5c;
                        border-radius: 2px;
                    }
                """)
    
    def send_scanControl(self):
        # # original name: send_robotControl
        # command = 1
        # start = b"RC"
        # send_data = start + command.to_bytes(1, byteorder="big") + b'\n'
        # try:
        #     self.sock.sendall(send_data)
        #     print(f"'{send_data}' has been sent to {self.server_ip}:{self.server_port}.")
        #     self.scanstart_btn.setEnabled(False)
        # except Exception as e:
        #     print(f"Error sending messages: {e}")
        print("scanstart")
        work_area = self.scan_area_combo.currentText()
        priority_area = self.scan_area_priority_combo.currentText()
        self.publish_node.publish_topic(topic_name="task_topic",task_type=0, area=work_area,priority_area=priority_area) # task_type 0: scan, 1: pollinate
        # self.db_topic_publisher = self.create_publisher(RequestTopic,'ClientRequestTopic', 10)
        # or publish topic
    def send_polliControl(self):
        print("pollistart")
        work_area = self.polli_area_combo.currentText()
        priority_area = self.polli_area_priority_combo.currentText()
        self.publish_node.publish_topic(topic_name="task_topic",task_type=1, area=work_area,priority_area=priority_area)
    
    def process_received_message(self, data):
        print(f"received message: {data}")
        if int.from_bytes(data[:1], byteorder="big") == 0:

            self.scanstart_btn.setEnabled(True) # set the 로봇 스캔시작 button to be ON
            # if error occurs, comment the bottom
            self.scanstart_btn.setStyleSheet("""
                QPushButton {
                    color: rgb(255, 255, 255);
                    background-color: rgb(167, 153, 139);
                }
                QPushButton:hover {
                    color: rgb(255, 255, 255);
                    background-color: rgb(148, 135, 123);
                }
            """)

    # ------------------------------------------------------------------------------------------------------------------------
    # statistics and anlytics (page 3)
    # ------------------------------------------------------------------------------------------------------------------------

    def pollinated_tree_count(self):
        # create list for y-axis
        y1 = [5, 25]
        
        # create horizontal list i.e x-axis
        x = [1, 2]  # Mapping "인공수분 완료" to 1 and "인공수분 미완료" to 2
        labels = {1: "수분 완료", 2: "수분 미완료"}  # Mapping positions to labels

        bargraph = BarGraphItem(x = x, height = y1, width = 0.6, brush ='#6e6053')
        plot = pyqtgraph.plot()
        plot.addItem(bargraph)
        plot.setBackground('#ede3d6') 

        x_axis = plot.getAxis('bottom')
        x_axis.setTicks([list(labels.items())])  # x-axis label
        x_axis.setStyle(tickFont=pyqtgraph.QtGui.QFont("NanumSquareRound", 9, QFont.Bold))  # Set font and color for x-ticks
        x_axis.setTextPen(pyqtgraph.mkPen('black'))  # Set tick label color

        styles = {"color": "black", "font-size": "12px", "font": "NanumSquareRound ExtraBold"}
        plot.getAxis('bottom').setLabel('인공 수분', **styles)  # x-axis label
        plot.getAxis('left').setLabel('나무 개수', **styles)  # y-axis label

        layout = QVBoxLayout()
        layout.addWidget(plot, alignment=Qt.AlignCenter)  # Align the chart to the center
        self.barchart_groupBox.setLayout(layout)

    def monthly_progress_chart(self):
            # Define the data sets
            set0 = QBarSet("개화된 꽃")
            set1 = QBarSet("꽃봉우리")

   
            # Assign values to the sets
            set0 << 10 << 20 << 30 << 25
            set1 << 30 << 20 << 10 << 20

            set0.setColor(QColor("#b8ac9d"))
            set1.setColor(QColor("#716352"))

            # Create the series and add the sets
            series = QBarSeries()
            series.append(set0)
            series.append(set1)

            # Create the chart
            chart = QChart()
            chart.addSeries(series)
            chart.setBackgroundBrush(QColor("#ede3d6"))
            #chart.setTitle("Monthly Data")
            chart.setAnimationOptions(QChart.SeriesAnimations)
            chart.legend().setAlignment(Qt.AlignRight)
            chart.legend().setVisible(True)  # Show the legend

            # Customize the axes
            categories = ["Jun", "July", "Aug", "Sep"]
            axisX = QBarCategoryAxis()
            axisX.append(categories)
            chart.addAxis(axisX, Qt.AlignBottom)
            series.attachAxis(axisX)

            axisY = QValueAxis()
            axisY.setRange(0, 40)
            axisY.setLabelFormat("%.0f")  # Remove decimal points in labels
            chart.addAxis(axisY, Qt.AlignLeft)
            series.attachAxis(axisY)

            # Create the chart view and set the chart
            chart_view = QChartView(chart, self.linechart_groupBox)
            chart_view.setRenderHint(QPainter.Antialiasing)
            chart_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            chart_view.setMinimumWidth(400)  # Ensures horizontal scroll when chart is too wide
            chart_view.setMinimumHeight(200)  # Ensures horizontal scroll when chart is too wide

            # Create a scroll area to make it scrollable horizontally
            scroll_area = QScrollArea(self.linechart_groupBox)
            scroll_area.setWidget(chart_view)
            scroll_area.setWidgetResizable(True)
            scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
            scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

            # Add the chart view to the group box layout
            layout = QVBoxLayout()
            layout.addWidget(chart_view, alignment=Qt.AlignCenter)
            layout.addWidget(scroll_area)
            layout.setContentsMargins(0, 0, 0, 0)
            self.linechart_groupBox.setLayout(layout)

    def display_log(self):
        combined_results = self.retrieve_from_database()
        
        self.logchart_table = QTableWidget()
        
        self.logchart_table.setRowCount(len(combined_results))  # Set rows based on the number of combined results
        self.logchart_table.setColumnCount(8)  # Set columns based on the combined fields: (Tree.id, location, planting_date, flower_count, bud_count, pollination_count, season)

        self.logchart_table.setHorizontalHeaderLabels(['Tree ID', 'X Coordinate', 'Z Coordinate', 'Flower Count', 'Bud Count', 'Pollination Count', 'Season', 'Planting Date'])
        self.logchart_table.resizeColumnsToContents()
        self.logchart_table.setFixedWidth(800)
        #self.logchart_table.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)

        for i, row in enumerate(combined_results):
            for j, data in enumerate(row):
                self.logchart_table.setItem(i, j, QTableWidgetItem(str(data)))
                self.logchart_table.setStyleSheet("QTableView::item:selected { color:white; background:#716352; font-weight:900; }"
                           "QTableCornerButton::section { background-color:#746858; }"
                           "QHeaderView::section { color:white; background-color:#746858; }")

        layout = QVBoxLayout()
        layout.addWidget(self.logchart_table, alignment=Qt.AlignCenter)
        self.logchart_groupBox.setLayout(layout)
        layout.setContentsMargins(20, 20, 20, 20)  # Adjust margins as necessary
        #layout.setSpacing(10)  # Adjust spacing between elements if needed

    # ------------------------------------------------------------------------------------------------------------------------
    # set schedule settings page 4
    # ------------------------------------------------------------------------------------------------------------------------


    def save_toggle_state(self, toggle_name, toggle_state):
        # state_file = "GUI/toggle_state.json"
        state_file = "applecare_gui_pkg/applecare_gui_pkg/toggle_state.json"
        if os.path.exists(state_file):
            with open(state_file, "r") as file:
                toggle_states = json.load(file)
        else:
            toggle_states = {}

        if toggle_state == 0:
            toggle_states[toggle_name] = True
        elif toggle_state == 1:
            toggle_states[toggle_name] = False

        with open(state_file, "w") as file:
            json.dump(toggle_states, file)

    def on_toggle_state_change(self, toggle, toggle_name):
        if toggle.isChecked(): # If the toggle is turned on
            toggle_state = 0 #turned on
            self.save_toggle_state(toggle_name, toggle_state)
            if toggle_name == 'Schedule 1':
                date = self.schedule1_timeedit
                toggle_no = 1
            elif toggle_name == 'Schedule 2':
                date = self.schedule2_timeedit
                toggle_no = 2
            elif toggle_name == 'Schedule 3':
                date = self.schedule3_timeedit
                toggle_no = 3
            selected_start_time = date.dateTime()
            selected_end_time = selected_start_time.addSecs(30 * 60) # 30*60 = 1800 seconds, 30 mins
            
            self.selected_start_str = selected_start_time.toString("HH:mm")
            selected_end_str = selected_end_time.toString("HH:mm")
            self.send_to_database(self.selected_start_str, selected_end_str) #send scheduled time to db
            print(f"{toggle_name} (status {toggle_state}) sent to DB with {self.selected_start_str}, {selected_end_str}")
            self.send_to_central_server(toggle_no, toggle_state)
            print(f"{toggle_name} (status {toggle_state}) requested to turn on the schedule time to Central Server at {self.selected_start_str}, {selected_end_str}")

        elif toggle.isChecked() == False:
            toggle_state = 1 #turned off
            self.save_toggle_state(toggle_name, toggle_state)
            if toggle_name == 'Schedule 1':
                toggle_no = 1
            elif toggle_name == 'Schedule 2':
                toggle_no = 2
            elif toggle_name == 'Schedule 3':
                toggle_no = 3
            self.send_to_central_server(toggle_no, toggle_state)

            print(f"{toggle_name} (status {toggle_state}) requested to turn off the schedule time to Central Server")
    
    def load_toggle_state(self):
        # state_file = "GUI/toggle_state.json"
        state_file = "applecare_gui_pkg/applecare_gui_pkg/toggle_state.json"
        if os.path.exists(state_file):
            with open(state_file, "r") as file:
                return json.load(file)
        return {}

    def onoff_schedule(self):
        toggle_states = self.load_toggle_state()

        schedule1_toggle = AnimatedToggle(
                checked_color="#ffbb00",
                pulse_checked_color="#44FFB000"
            )
        
        schedule2_toggle = AnimatedToggle(
                checked_color="#ffbb00",
                pulse_checked_color="#44FFB000"
            )
        schedule3_toggle = AnimatedToggle(
                checked_color="#ffbb00",
                pulse_checked_color="#44FFB000"
            )
        # -------------------------------------------
        # 구역 우선순위 토글 10.29
        schedule1_area_toggle = AnimatedToggle(
                checked_color="#ffbb00",
                pulse_checked_color="#44FFB000"
            )
        schedule2_area_toggle = AnimatedToggle(
                checked_color="#ffbb00",
                pulse_checked_color="#44FFB000"
            )
        schedule3_area_toggle = AnimatedToggle(
                checked_color="#ffbb00",
                pulse_checked_color="#44FFB000"
            )
        # Restore previous toggle states 구역 관련 #### 상태 변화에 따른 함수 연결 없음. 근데 필요하나...? 몰루
        schedule1_area_toggle.setChecked(toggle_states.get('Schedule 1 area', False))
        schedule2_area_toggle.setChecked(toggle_states.get('Schedule 2 area', False))
        schedule3_area_toggle.setChecked(toggle_states.get('Schedule 3 area', False))

        sche1_area_layout = QVBoxLayout(self.schedule1_area_toggle)
        sche1_area_layout.addWidget(schedule1_area_toggle)
        sche1_area_layout.setAlignment(Qt.AlignCenter)

        sche2_area_layout = QVBoxLayout(self.schedule2_area_toggle) 
        sche2_area_layout.addWidget(schedule2_area_toggle)
        sche2_area_layout.setAlignment(Qt.AlignCenter)

        sche3_area_layout = QVBoxLayout(self.schedule3_area_toggle)
        sche3_area_layout.addWidget(schedule3_area_toggle)
        sche3_area_layout.setAlignment(Qt.AlignCenter)


        sche1_area_layout.setContentsMargins(0, 0, 0, 0)
        sche2_area_layout.setContentsMargins(0, 0, 0, 0)
        sche3_area_layout.setContentsMargins(0, 0, 0, 0)
        # -------------------------------------------
        

        # Restore previous toggle states
        schedule1_toggle.setChecked(toggle_states.get('Schedule 1', False))
        schedule2_toggle.setChecked(toggle_states.get('Schedule 2', False))
        schedule3_toggle.setChecked(toggle_states.get('Schedule 3', False))

        # Connect the toggles to the state change signal
        schedule1_toggle.stateChanged.connect(lambda: self.on_toggle_state_change(schedule1_toggle, 'Schedule 1'))
        schedule2_toggle.stateChanged.connect(lambda: self.on_toggle_state_change(schedule2_toggle, 'Schedule 2'))
        schedule3_toggle.stateChanged.connect(lambda: self.on_toggle_state_change(schedule3_toggle, 'Schedule 3'))

        sche1_layout = QVBoxLayout(self.schedule1_toggle)
        sche1_layout.addWidget(schedule1_toggle)
        sche1_layout.setAlignment(Qt.AlignCenter)

        sche2_layout = QVBoxLayout(self.schedule2_toggle) 
        sche2_layout.addWidget(schedule2_toggle)
        sche2_layout.setAlignment(Qt.AlignCenter)

        sche3_layout = QVBoxLayout(self.schedule3_toggle)
        sche3_layout.addWidget(schedule3_toggle)
        sche3_layout.setAlignment(Qt.AlignCenter)


        sche1_layout.setContentsMargins(0, 0, 0, 0)
        sche2_layout.setContentsMargins(0, 0, 0, 0)
        sche3_layout.setContentsMargins(0, 0, 0, 0)

    # ------------------------------------------------------------------------------------------------------------------------
    # communication with central server & DB
    # ------------------------------------------------------------------------------------------------------------------------

    def send_to_central_server(self, toggle_no, toggle_state):
        try:
            toggle_no=str(toggle_no)
            toggle_state=str(toggle_state)
            toggle_no_byte = toggle_no.encode('utf-8')
            toggle_state_byte = toggle_state.encode('utf-8')
            if toggle_state == 0:
                schedule_time = self.selected_start_str.encode('utf-8')
                # print(len(schedule_time))
                self.sock.sendall(b'SS' + toggle_state_byte + toggle_no_byte + schedule_time + b'\n')
                print(f"sent schedule{toggle_no} (status {toggle_state} to server: raw-{self.selected_start_str} packet-{schedule_time}")
            elif toggle_state == 1:
                self.sock.sendall(b'SS' + toggle_state_byte + toggle_no_byte + b'\n')
                print(f"sent schedule{toggle_no} (status {toggle_state} to server: toggle down (switched off)")
        except Exception as e:
            print(f"Error in communication: {e}")

    # send data to TaskSchedule table on MySQL database
    def send_to_database(self, start_time, end_time):
        # try:
        #     # Replace with your database connection details
        #     connection = mysql.connector.connect(

        #         host='192.168.0.59',
        #         port = 3306, 
        #         user='lkm',
        #         password='1234',

        #         # host='localhost',
        #         # port = 3306, 
        #         # user='root',
        #         # password='1',

        #         database='AppleCareDB'
        #     )
        #     cursor = connection.cursor()
        #     print(f"Connection Success")
            
        #     # Insert or update query based on your table structure
        #     query = f"INSERT INTO TaskSchedule (start, end, robot_id) VALUES ('{start_time}', '{end_time}', 1)"
        #     cursor.execute(query)
        #     connection.commit()
        #     print(f"Time sent to database: {start_time}, {end_time}")
        # except mysql.connector.Error as err:
        #     print(f"Error in DB connection: {err}")
        # finally:
        #     if connection.is_connected():
        #         cursor.close()
        #         connection.close()
        #         print(f"Connection closed")
        self.connect_to_database()
        # Insert or update query based on your table structure
        query = f"INSERT INTO TaskSchedule (start, end, robot_id) VALUES ('{start_time}', '{end_time}', 1)"
        self.cursor.execute(query)
        self.connection.commit()
        print(f"Time sent to database: {start_time}, {end_time}")

    # retrieve data from Tree and TreeCurrentStatus table on MySQL database
    def retrieve_from_database(self):
        combined_results = []
        # try:
            # Replace with your database connection details
        #     connection = mysql.connector.connect(

        #         host='192.168.0.59',
        #         port = 3306, 
        #         user='lkm',
        #         password='1234',

        #         # host='localhost',
        #         # port = 3306, 
        #         # user='root',
        #         # password='1',

        #         database='AppleCareDB'
        #     )
        #     cursor = connection.cursor()
        #     print(f"DB Connection Success")
            
        #     cursor.execute("SELECT id, x_coordinate, z_coordinate, planting_date FROM Tree")
        #     Tree_result = cursor.fetchall()
        #     #print(f"Recieved row from DB: Tree")

        #     cursor.execute("SELECT id, tree_id, flower_count, bud_count, pollination_count, season FROM TreeCurrentStatus")
        #     TreeCurrentStatus_result = cursor.fetchall()
        #     #print(f"Recieved row from DB: TreeCurrentStatus")

        #     for tree in Tree_result:
        #         for status in TreeCurrentStatus_result:
        #             if tree[0] == status[1]:
        #                 combined_results.append((
        #                     tree[0], # tree_id
        #                     tree[1], # location x_coordinate
        #                     tree[2], # location z_coordinate
        #                     status[2], # flower_count
        #                     status[3], # bud_count
        #                     status[4], # pollination_count
        #                     status[5], # season
        #                     tree[3] #planting_date
        #                 ))

        #     connection.commit()
        # except mysql.connector.Error as err:
        #     print(f"Error in MySQL connection: {err}")
        # finally:
        #     if connection.is_connected():
        #         cursor.close()
        #         connection.close()
        #         print(f"DB Connection closed")
        # return combined_results

        self.connect_to_database()
        self.cursor.execute("SELECT id, x_coordinate, z_coordinate, planting_date FROM Tree")
        Tree_result = self.cursor.fetchall()
        #print(f"Recieved row from DB: Tree")

        self.cursor.execute("SELECT id, tree_id, flower_count, bud_count, pollination_count, season FROM TreeCurrentStatus")
        TreeCurrentStatus_result = self.cursor.fetchall()
        #print(f"Recieved row from DB: TreeCurrentStatus")

        for tree in Tree_result:
            for status in TreeCurrentStatus_result:
                if tree[0] == status[1]:
                    combined_results.append((
                        tree[0], # tree_id
                        tree[1], # location x_coordinate
                        tree[2], # location z_coordinate
                        status[2], # flower_count
                        status[3], # bud_count
                        status[4], # pollination_count
                        status[5], # season
                        tree[3] #planting_date
                    ))

        self.connection.commit()
        return combined_results

    
    def connect_to_database(self):
        if not self.connection_to_database:
            self.connection =mysql.connector.connect(

                    # host='192.168.0.57',
                    # port = 3306, 
                    # user='lkm',
                    # password='1234',

                    host='localhost',
                    port = 3306, 
                    user='root',
                    password='1',

                    database='AppleCareDB'
                )
            self.cursor = self.connection.cursor()
            print(f"DB Connection Success")
            self.connection_to_database = True
        else:
            # print("already connected")
            pass
import threading
from rclpy.executors import MultiThreadedExecutor
# def spin_thread(NODE):
#     rclpy.spin(NODE)
# 다중 노드
def spin_thread(executor):
    executor.spin()


def main():
    try:
        rclpy.init()
        Imagenode = ImageSubscriber()
        # # 스레드 생성 및 실행
        # spin_thread_handle = threading.Thread(target=spin_thread, args=(Imagenode,))
        # spin_thread_handle.start()

        amclNode = amclSubscriber()
        # spin_amcl_thread_handle = threading.Thread(target=spin_thread, args=(amclNode,))
        # spin_amcl_thread_handle.start()
        topicPublisher=GuiTopicPublisher()
        # MultiThreadedExecutor 생성
        executor = MultiThreadedExecutor()
        executor.add_node(Imagenode)
        executor.add_node(amclNode)
        executor.add_node(topicPublisher)
        # 스레드 생성 및 실행
        spin_thread_handle = threading.Thread(target=spin_thread, args=(executor,))
        spin_thread_handle.start()

        app = QApplication([])
        window = OrchardGUI(video_node=Imagenode,amcl_node=amclNode,publish_node=topicPublisher)
        window.show()
        app.exec_()
    finally:
        Imagenode.destroy_node()
        amclNode.destroy_node()
        rclpy.shutdown()
        # cv2.destroyAllWindows()
        if window.connection.is_connected():
            print(window.connection.is_connected())
            window.cursor.close()
            window.connection.close()
            print(window.connection.is_connected())
            print(f"DB Connection closed")

if __name__ == "__main__":
    main()

