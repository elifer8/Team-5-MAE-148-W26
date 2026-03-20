import roboflow
# Use your same API key
rf = roboflow.Roboflow(api_key="UOi4p4RmZsJV6kqNi8ln")
project = rf.workspace().project("traffic-light-training")
version = project.version(6)

# This downloads a .zip containing the .blob file and everything else
version.download("yolov8")