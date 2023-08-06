from pathlib import Path
from ROAR.control_module.section_pid_controller import SectionalPID


test_file_path = Path("./ROAR/datasets/control/sectional_pid/test")
with open(test_file_path, 'r') as test_file:
    test1 = SectionalPID(test_file)
    print(test1.array)