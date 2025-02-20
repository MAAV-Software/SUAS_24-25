import random
import math
import pyproj
import chevron
import pymap3d as pm
starting_coord = (0, 0, 0) # degree starting position plus height
# total_way_points = 10
total_distance = 3/69 # 3 miles in degrees (1 degree = 69 miles)
minHeight = 100 # agl or would be 217 feet msl
maxHeight = 125
way_points = []
way_points.append(starting_coord)

#generate a random number between 5-10
howMany = random.randint(5, 10)
step = total_distance/howMany
variance = 0.00001
distance = 0
for i in range(1, howMany+1): #run through are total points
    # x = random.uniform(-total_distance/2, total_distance/2) #generate a random x and y coordinate that is within the total distance remaining
    # y = random.uniform(-total_distance/2, total_distance/2)
    radius = step
    angle = random.uniform(0, 2*3.14159)
    long = way_points[i-1][0]+ (radius * math.cos(angle))
    lat =  way_points[i-1][1]+ radius * math.sin(angle)

    height = random.uniform(minHeight, maxHeight)
    way_points.append((long, lat, height))
    #subtract the distance between the two points from the total distance so that we stay within 10
    distance += (((abs(long)-abs(way_points[i-1][0]))**2 + 
                  (abs(lat)-abs(way_points[i-1][1]))**2)**0.5)


f = open("way_points.txt", "w")
f.write("Total path length: " + str(distance*69) + " miles\n")
f.write("Total points: " + str(len(way_points)) + "\n")
for i in range(0, len(way_points)):
    f.write(str(way_points[i][0]) + " " + str(way_points[i][1]) + " " + str(way_points[i][2]) + "\n")
f.close()


f = open("waypoints.sdf", "w")


source_proj = pyproj.Proj(init='epsg:4326')  # WGS84
target_proj = pyproj.Proj(init='epsg:3857') 
with open('templates/world_template.mustache', 'r') as m:
    rendered_content = chevron.render(m)
    f.write(rendered_content)
    # print(rendered_content)
    
lat0 = 0
lon0 = 0
h0 = 0

for i in range(1, len(way_points)):
    lat = way_points[i][0]
    lon = way_points[i][1]
    h = way_points[i][2]
    new_point = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0)
    # x, y = pyproj.transform(source_proj, target_proj, way_points[i][0], way_points[i][1])
    with open('templates/waypoint_template.mustache', 'r') as m:
        rendered_content = chevron.render(m, {'point': (i+1), 'x': new_point[0], 'y': new_point[1], 'z': new_point[2]*.3048})
        f.write(rendered_content)
    # f.write("\t<pose>" + str(25) + " " + str(25) + " " + str(25) + " 0 -0 0</pose>\n")
    #project the lat and long to x and y using WGS84 projection

f.write("</world>\n")
f.write("</sdf>\n")
