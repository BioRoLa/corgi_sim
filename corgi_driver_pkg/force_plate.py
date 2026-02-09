from controller import Robot
import csv
import os

class ForcePlateDriver:
    def __init__(self, webots_node=None, properties=None):
        # Support both plugin mode (webots_node) and standalone mode
        self.robot = webots_node if webots_node is not None else Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.force_plates = [self.robot.getDevice(f'force_plate_{i}') for i in range(1, 5)]
        for plate in self.force_plates:
            plate.enable(self.timestep)
        
        self.csv_path = os.path.join(os.getenv('HOME'), 'corgi_ws/corgi_ros_ws/output_data/sim_force_plate.csv')
        self._initialize_csv()
        
    def _initialize_csv(self):
        """Initialize CSV file with header columns"""
        columns = [f'{axis}_{i}' for i in range(1, 5) for axis in ['Fx', 'Fy', 'Fz']]
        
        for label in ['O1', 'O2', 'O3', 'O4', 'Trigger']:
            columns.extend([f'{label}_x', f'{label}_y', f'{label}_z'])
            
        columns.extend(['vicon_pos_x', 'vicon_pos_z', 'vicon_vel_x', 'vicon_vel_z', 'vicon_roll', 'vicon_pitch'])
        
        with open(self.csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(columns)
    
    def step(self):
        """Collect force plate data and write to CSV"""
        row = [value for plate in self.force_plates for value in plate.getValues()[:3]]
        
        for i in range(len(self.force_plates)):
            row[i*3+2] -= 9.81
            row[i*3+2] *= -1
        
        with open(self.csv_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(row)
    
    def run(self):
        """Main loop for standalone execution"""
        while self.robot.step(self.timestep) != -1:
            self.step()


def main(args=None):
    """Entry point for ROS2 node"""
    driver = ForcePlateDriver()
    driver.run()

if __name__ == "__main__":
    driver = ForcePlateDriver()
    driver.run()

