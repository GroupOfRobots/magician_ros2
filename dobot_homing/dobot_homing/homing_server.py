from dobot_msgs.srv import ExecuteHomingProcedure
import rclpy
from rclpy.node import Node
from dobot_driver.dobot_handle import bot
import time
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter



class HomingService(Node):

    def __init__(self):
        super().__init__('dobot_homing_srv')
        self.srv = self.create_service(ExecuteHomingProcedure, 'dobot_homing_service', self.homing_callback)
        self.srv = self.create_service(GetParameters, 'get_homing_status', self.get_homing_status_callback)
        self.instruction = "The homing procedure has started. Wait until the arm stops moving and the led stops flashing blue and turns green."
        self.parameter_descriptor = ParameterDescriptor(description='Position where Dobot finishes homing procedure expressed in Cartesian coordinates x, y, z, r.', additional_constraints = "Target position must be within Dobot's range! The unit is millimeters.")
        self.homing_status_descriptor = ParameterDescriptor(description='Status indicating whether Dobot has finished homing prodecure.', additional_constraints = "Possible values are \"unattempted\" and \"finished\"")
        self.declare_parameter('homing_position', rclpy.Parameter.Type.DOUBLE_ARRAY, self.parameter_descriptor)
        self.declare_parameter('homing_status', rclpy.Parameter.Type.STRING, self.homing_status_descriptor)

        self.add_on_set_parameters_callback(self.parameters_callback)


    def parameters_callback(self, params):

        for param in params:
            if param.name == 'homing_position' and param.type_ == Parameter.Type.DOUBLE_ARRAY:
                bot.set_homing_parameters(param.value[0], param.value[1], param.value[2], param.value[3])
                return SetParametersResult(successful=True)
            elif param.name == 'homing_status' and param.type_ == Parameter.Type.STRING:
                return SetParametersResult(successful=True)
            else:
                return SetParametersResult(successful=False)


    def get_homing_status_callback(self, request, response):
        param = self.get_parameter('homing_status')
        response.values = [param.get_parameter_value()]
        return response


    def homing_callback(self, request, response):

        bot.set_homing_command(0) 
        time.sleep(0.1)

        self.set_parameters([rclpy.parameter.Parameter('homing_status', rclpy.Parameter.Type.STRING, 'finished')])

        response.success = True
        response.instruction = self.instruction
        return response




def main(args=None):
    rclpy.init(args=args)

    minimal_service = HomingService()

    try:
        rclpy.spin(minimal_service)
    except (KeyboardInterrupt):
        pass
    finally:
        minimal_service.destroy_node()


if __name__ == '__main__':
    main()