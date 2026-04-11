import os
import glob

def replace_in_file(file_path):
    with open(file_path, 'r') as f:
        content = f.read()
    
    new_content = content.replace('ne_aim_traj.hpp', 'ne_aim_state.hpp')
    new_content = new_content.replace('NeAimTraj_t', 'NeAimState_t')
    new_content = new_content.replace('aim_traj_c_sPtr', 'aim_state_c_sPtr')
    new_content = new_content.replace('aim_trajs', 'aim_states')
    new_content = new_content.replace('aim_traj', 'aim_state')
    new_content = new_content.replace('NeAimTrajCSPtr_t', 'NeAimStateCSPtr_t')

    if content != new_content:
        with open(file_path, 'w') as f:
            f.write(new_content)
        print(f"Updated {file_path}")

for root, _, files in os.walk('/Users/ziyu/codes/ne_vision/ne_vision'):
    for file in files:
        if file.endswith(('.cpp', '.hpp')):
            replace_in_file(os.path.join(root, file))

