import json

# Input and output file names
input_file = "milan_flight_path.txt"
output_file = "qgc_points.plan"

# Read waypoints from the text file
with open(input_file, "r") as f:
    lines = f.readlines()

# Extract waypoint data, skipping the first two lines (header)
waypoints = [line.strip().split() for line in lines[2:]]

# Convert to float values
FEET_TO_METERS = 0.3048
waypoints = [(float(lat), float(lon), (float(alt)*FEET_TO_METERS)) for lat, lon, alt in waypoints]

# Define QGroundControl mission format (Version 1)
mission_plan = {
    "fileType": "Plan",
    "version": 1,  # Set to version 1 for compatibility
    "groundStation": "QGroundControl",
    "mission": {
        "firmwareType": 12,
        "vehicleType": 20,
        "cruiseSpeed": 5,
        "hoverSpeed": 2,
        "globalPlanAltitudeMode": 0,
        "items": [],
        "plannedHomePosition": [waypoints[0][0], waypoints[0][1], waypoints[0][2]]
    },
    "geoFence": {  # Required geoFence key
        "polygons": [],
        "circles": [],
        "version": 2
    },
    "rallyPoints": {  # Required rallyPoints key
        "points": [],
        "version": 2
    }
}

# # Add waypoints
# # Worried about all these hard coded jump ids :skull:
# mission_plan["mission"]["items"].append({
#         "AMSLAltAboveTerrain": None,
#         "Altitude": waypoints,
#         "AltitudeMode": 1,
#         "autoContinue": True,
#         "command": 84,
#         "doJumpId": 1,
#         "frame": 3,
#         "params": [0, 1, 0, None, lat, lon, alt],
#         "type": "SimpleItem"
#     })

for i, (lat, lon, alt) in enumerate(waypoints):
    mission_plan["mission"]["items"].append({
        "AMSLAltAboveTerrain": None,
        "Altitude": alt,
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": 84 if i == 0 else 16,  # First waypoint = Takeoff
        "doJumpId": i + 1,
        "frame": 3,
        "params": [0, 0, 0, None, lat, lon, alt],
        "type": "SimpleItem"
    })

mission_plan["mission"]["items"].append({
                "altitudesAreRelative": True,
                "complexItemType": "vtolLandingPattern",
                "landCoordinate": [
                    waypoints[0][0],
                    waypoints[0][1],
                    0
                ],
                "landingApproachCoordinate": [
                    waypoints[0][0],
                    waypoints[0][1],
                    30 # hardcoded altitude
                ],
                "loiterClockwise": True,
                "loiterRadius": 75,
                "stopTakingPhotos": True,
                "stopVideoPhotos": True,
                "type": "ComplexItem",
                "useLoiterToAlt": True,
                "version": 1
            })


# Save as .plan file
with open(output_file, "w") as f:
    json.dump(mission_plan, f, indent=4)

print(f"Mission plan saved as {output_file} (!!! TAKING milan_points.txt AS INPUT !!!).")
