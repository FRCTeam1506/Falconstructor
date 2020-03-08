import json
import os

name = input("Name of JSON file (without the .json): ")

interior = []

with open(f"{os.getcwd()}\\src\\main\\deploy\\output\\{name}.wpilib.json", "r") as f:
    data = json.load(f)

start = data[0]

for i in range(1, len(data)-1, 5):
    interior.append(data[i])
    
# mid = data[ (len(data) // 2) + 1 ]
end = data[-1]

start_x = start['pose']['translation']['x']
start_y = start['pose']['translation']['y']
start_radians = start['pose']['rotation']['radians']

# mid_x = mid['pose']['translation']['x']
# mid_y = mid['pose']['translation']['y']
# mid_radians = mid['pose']['rotation']['radians']

end_x = end['pose']['translation']['x']
end_y = end['pose']['translation']['y']
end_radians = end['pose']['rotation']['radians']

print(f"\n********START********")
print(f"    start X: {start_x}")
print(f"    start Y: {start_y}")
print(f"    start Radians: {start_radians}")

print(f"\n*********MID*********")
for i, dat in enumerate(interior):
    print(f"    mid X {i+1}: {dat['pose']['translation']['x']}")
    print(f"    mid Y {i+1}: {dat['pose']['translation']['y']}")
    print(f"    mid Radians {i+1}: {dat['pose']['rotation']['radians']}\n")

print(f"\n*********END*********")
print(f"    end X: {end_x}")
print(f"    end Y: {end_y}")
print(f"    end Radians: {end_radians}")

print(f"\n*****CODE SAMPLE*****")
print(f"""        
        var start = new Pose2d(new Translation2d({start_x}, {start_y}), new Rotation2d({start_radians}));
        var end = new Pose2d(new Translation2d({end_x}, {end_y}), new Rotation2d({end_radians}));

        var interiorWaypoints = new ArrayList<Translation2d>();""")

for i in interior:
    print(f"""        interiorWaypoints.add(new Translation2d({i['pose']['translation']['x']}, {i['pose']['translation']['y']}));""")