rm -f noir.sdf
rosrun xacro xacro -o  noir_base.urdf noir_description/urdf/noir_base.xacro enable_mavlink_interface:=true enable_ground_truth:=true enable_wind:=false enable_logging:=true rotors_description_dir:=../
gz sdf -p  noir_base.urdf >> noir.sdf
rm -f noir_base.urdf
