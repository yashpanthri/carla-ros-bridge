# A minimal message: timestamp + reference frame + metric depth value
std_msgs/Header header
float32 depth   # metres
string object_id # object id(Car,Pedestrian,Bicycle, etc)  --- CursorAI
