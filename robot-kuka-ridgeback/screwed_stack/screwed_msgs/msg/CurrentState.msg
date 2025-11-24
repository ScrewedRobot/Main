Header header

int8 FAILED=0       # Task failed, stop execution - (Operator intervention required)
int8 HOME=1         # Manipulator is at home position
int8 STANDBY=2      # Manipulator is ready to begin - (Operator intervention required)
int8 ACQUIRE=3  # Manipulator is acquiring goal poses
int8 APPROACH=4     # EE hovers over the target
int8 INSERTION=5    # Spindle not active yet, EE lowers
int8 SCREWING=6     # Spindle active, EE lowers - (Operator intervention required)
int8 COMPLETE=7   # Task is complete, move to next task - (Operator intervention required)
int8 STOPPED=8      # Manipulator is paused by operator
int8 NAVIGATING=9
int8 ARRIVED=10
int8 current_state

bool is_frozen
bool is_manual


Task[] task_list