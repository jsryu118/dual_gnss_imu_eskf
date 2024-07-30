import matplotlib.pyplot as plt
import os

# 현재 스크립트의 디렉토리 경로를 가져오기
script_dir = os.path.dirname(os.path.abspath(__file__))

# odom_log.txt 파일 경로를 설정
file_path = os.path.join(script_dir, 'odom_log.txt')

# 데이터 읽기
data = []
with open(file_path, 'r') as file:
    for line in file:
        parts = line.strip().split()
        color = int(parts[0])
        x = float(parts[1])
        y = float(parts[2])
        data.append((color, x, y))

# 색상 설정
color_map = {0: 'red', 1: 'blue', 2: 'purple', 3: 'green'}
colors = [color_map.get(point[0], 'black') for point in data]

# x, y 좌표 추출
x_coords = [point[1] for point in data]
y_coords = [point[2] for point in data]

# 시각화
plt.figure(figsize=(10, 6))

# 포인트 플로팅
plt.scatter(x_coords, y_coords, c=colors)

# 축 레이블 설정
plt.xlabel('X Label')
plt.ylabel('Y Label')

plt.show()
