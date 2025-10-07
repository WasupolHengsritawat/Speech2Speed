#!/home/absolutezeno/physical_ai/venv/bin/python3

import rclpy
from rclpy.node import Node
from langchain.agents import initialize_agent, AgentType
from langchain.tools import tool

from speech2speed_interface.srv import TwistTraj, String
from speech2speed_interface.msg import TwistSimpleStamped
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

from langgraph.prebuilt import create_react_agent
from langchain_openai import ChatOpenAI

from speech2speed.llm import HFChatWrapper

from langchain_core.output_parsers import StrOutputParser

# run this before running this script:
# chmod +x ~/physical_ai/ws/install/speech2speed/lib/speech2speed/agent.py

# ===================== LangChain Tool =====================
def make_call_traj_service(node):
    @tool
    def call_traj_service(vectors: str) -> str:
        """ Calls the ROS2 service name "/Traj" to send a trajectory. 
            This function will return success or error message according to the service response.

            Input: string of vectors in format 'time1:v_x1,v_y1,v_z1,w_x1,w_y1,w_z1 time2:v_x2,v_y2,v_z2,w_x2,w_y2,w_z2; ...' 
            Example Input: '0.0:1.0,2.0,3.0,0.1,0.2,0.3; 1.0:7.0,8.0,9.0,0.4,0.5,0.6'

            Output: success or error message"""
        try:
            vector_list = []
            for item in vectors.split(';'):
                item = item.strip()
                if not item:
                    continue
                time_str, coords = item.split(':')
                vx_str, vy_str, vz_str, wx_str, wy_str, wz_str = coords.split(',')
                vector_list.append((float(time_str), float(vx_str), float(vy_str), float(vz_str), float(wx_str), float(wy_str), float(wz_str)))

            req = TwistTraj.Request()
            for t, vx, vy, vz, wx, wy, wz in vector_list:
                msg = TwistSimpleStamped()
                msg.time = Float32(data=t)
                msg.linear = Vector3(x=vx, y=vy, z=vz)
                msg.angular = Vector3(x=wx, y=wy, z=wz)
                req.twist_traj.append(msg)

            # Use the node's client directly
            future = node.scheduler_client.call_async(req)

            # return f"Service call failed."

            if future.success():
                return f"Service call successful: {future.result()}"
            else:
                return f"Service call failed: {future.exception()}"
        except Exception as e:
            return f"Error calling /Traj service: {str(e)}"
    return call_traj_service


class AgentNode(Node):
    instance = None  # keep reference for tools
    scheduler_client = None

    def __init__(self):
        super().__init__('agent_node')
        AgentNode.instance = self

        # Create services/clients =======================================
        self.create_service(String, 'Prompt', self.prompt_callback)
        AgentNode.scheduler_client = self.create_client(TwistTraj, 'Traj')

        # Initialize LLM ================================================
        self.llm = ChatOpenAI(model="gpt-5-nano", 
                              api_key="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")

        # Register tools
        self.call_traj_tool = make_call_traj_service(self)
        tools = [self.call_traj_tool]
        self.agent = create_react_agent(self.llm, tools)

    def prompt_callback(self, req, res):
        self.get_logger().info(f"Received message: {req.prompt}")
        res = String.Response()

        try:
            result = self.agent.invoke({"messages": req.prompt})
            self.get_logger().info(f"Agent raw output: {result}")
            res.response = result['messages'][-1].content  # Extract the content of the last message
        except Exception as e:
            res.response = f"Error: {str(e)}"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
