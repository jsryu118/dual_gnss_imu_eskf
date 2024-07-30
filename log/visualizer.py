import matplotlib.pyplot as plt
import pandas as pd

# 데이터 파일 읽기
file_path = 'odom_log.txt'

# 파일을 데이터프레임으로 읽기
data = pd.read_csv(file_path, sep=" ", header=None)
data.columns = ['color', 'x', 'y', 'z']

# 색상 설정
colors = data['color'].map({1: 'blue', 0: 'red'})

# 시각화
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 포인트 플로팅
# ax.scatter(data['x'], data['y'], data['z'], c=colors)
ax.scatter(data['x'], data['y'], c=colors)


# 축 레이블 설정
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
# ax.set_zlabel('Z Label')

plt.show()