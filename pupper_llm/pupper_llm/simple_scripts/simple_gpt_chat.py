import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
GPT4_PROMPT = "Pretend that you are Pupper V3, a robotic AI-powered dog being constructed in Stanford’s CS123 lab. You have the following commands available in our Pupper API: [ MOVE, TURN_LEFT, TURN_RIGHT, BARK ]. MOVE moves the robot forward, in the direction it is currently facing, by about 5 feet. TURN_LEFT turns Pupper to the left by roughly 120deg. The same for TURN_RIGHT, but the other way. BARK causes Pupper to play an audio of a dog barking. No actions can be taken simultaneously.\
For the rest of this conversation, you are PupperV3. Please respond to all of the following requests using only a list of Pupper API commands, as if you were the physical manifestation of the robot. I must reiterate, you can ONLY respond using the provided API commands.\
Example output: 'TURN_LEFT,TURN_LEFT,BARK'\
Thank you for your help!"
API_KEY = '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'

client = OpenAI(api_key=API_KEY)

class GPT4ConversationNode(Node):
    def __init__(self):
        super().__init__('gpt4_conversation_node')

        # Create a subscriber to listen to user queries
        self.subscription = self.create_subscription(
            String,
            'user_query_topic',  # Replace with your topic name for queries
            self.query_callback,
            10
        )

        # Create a publisher to send back responses
        self.publisher_ = self.create_publisher(
            String,
            'gpt4_response_topic',  # Replace with your topic name for responses
            10
        )

        self.get_logger().info('GPT-4 conversation node started and waiting for queries...')


    
    # TODO: Implement the query_callback method
    # msg is a String message object that contains the user query. You can extract the query using msg.data
    def query_callback(self, msg):
        # Extract the user query from the message using the data attribute of message
        user_query = msg.data
        # Call GPT-4 API to get the response. Use the get_gpt4_response method and pass in the query
        gpt4_response = self.get_gpt4_response(user_query)
    
        # Publish the response (as the data to a String message) using self.publisher_ and its publish method, 
        response = String()
        response.data = gpt4_response

        # Publish the response to the ROS2 topic
        self.publisher_.publish(response)

        # DEBUG LOGGERS: Uncomment the following line to print the query and response (you may have to change the variable names)
        
        self.get_logger().info(f"Received user query: {user_query}") 
        self.get_logger().info(f"Published GPT-4 response: {response}")

    def get_gpt4_response(self, query):
        try:
            # Making the API call to GPT-4o using OpenAI's Python client
            prompt = GPT4_PROMPT # TODO
            response = client.chat.completions.create(model="gpt-4",  # Model identifier, assuming GPT-4 is used
            messages=[
                {"role": "system", "content": prompt},
                {"role": "user", "content": query}
            ],
            max_tokens=150)  # Adjust token limit based on your requirement)

            # Extract the assistant's reply from the response
            gpt4_response = response.choices[0].message.content
            return gpt4_response

        except Exception as e:
            self.get_logger().error(f"Error calling GPT-4 API: {str(e)}")
            return "Sorry, I couldn't process your request due to an error."

def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin it
    gpt4_conversation_node = GPT4ConversationNode()
    rclpy.spin(gpt4_conversation_node)

    # Clean up and shutdown
    gpt4_conversation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
