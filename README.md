# eskf_gnss_imu

IMU의 z축이 아래를 향하고 있어도 결과로 나오는 odom은 z축이 하늘을 보게 나온다.
<p align="center">
  <img src="imgs/result_figure.jpeg"/>
</p>

imu data를 그에 맞게 처리하고 있음.

여기서 imu_frame에서의 gnss, base_link의 위치는 z축이 위를 보는 프레임으로 가정하고 파라미터를 설정해야한다.
