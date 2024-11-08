import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
from openai import OpenAI
import karel  # Importing your KarelPupper API
GPT4_PROMPT = "Pretend that you are translating from human commander to Pupper V3 action sequences, a robotic AI-powered dog being constructed in Stanford’s CS123 lab. You have the following commands available in our Pupper API: [ MOVE, TURN_LEFT, TURN_RIGHT, BARK ]. MOVE moves the robot forward, in the direction it is currently facing, by about 5 feet. TURN_LEFT turns Pupper to the left by roughly 120deg. The same for TURN_RIGHT, but the other way. BARK causes Pupper to play an audio of a dog barking. No actions can be taken simultaneously.\
For the rest of this conversation, you are PupperV3. Please respond to all of the following requests using only a list of Pupper API commands, as if you were the physical manifestation of the robot. I must reiterate, you can ONLY respond using the provided API commands.\
Example output: 'TURN_LEFT,TURN_LEFT,BARK'\
Thank you for your help!"
# GPT4_PROMPT = "I am a robotic dog. I have the following APIs to call: \
#                'turn_left', 'turn_right', 'move', 'stop', 'bark'. \
#                Please call one of the APIs based on the input. \
#                For example, if you receive 'go_left', please output 'turn_left'. "
API_KEY = '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'

client = OpenAI(api_key=API_KEY)  # Set your OpenAI API key here

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

        # Initialize the text-to-speech engine
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)  # Set the speed of speech (optional)

        # Initialize KarelPupper robot control
        self.pupper = karel.KarelPupper()

    # TODO: Implement the query_callback method
    # msg is a String message object that contains the user query. You can extract the query using msg.data
    def query_callback(self, msg):
        # Paste in your implementation from simple_gpt_chat.py
        # Extract the user query from the message using the data attribute of message
        user_query = msg.data
        # Call GPT-4 API to get the response. Use the get_gpt4_response method and pass in the query
        gpt4_response = self.get_gpt4_response(user_query)
        print(
            f"gpt4 response {gpt4_response}"
        )
        # Publish the response (as the data to a String message) using self.publisher_ and its publish method, 
        response = String()
        response.data = gpt4_response

        # Publish the response to the ROS2 topic
        self.publisher_.publish(response)
        
        # Play the response through the speaker with the play_response method
        self.play_response(response)
        # Parse and execute robot commands if present with the execute_robot_command method
        self.execute_robot_command(response)

    def get_gpt4_response(self, query):
        try:
            # Making the API call to GPT-4 using OpenAI's Python client
            prompt = GPT4_PROMPT # TODO
            response = client.chat.completions.create(model="gpt-4",  # Model identifier, assuming GPT-4 is used
            messages=[
                {"role": "system", "content": prompt},
                {"role": "user", "content": query}
            ],
            max_tokens=150)  # Adjust token limit based on your requirement

            # Extract the assistant's reply from the response
            gpt4_response = response.choices[0].message.content
            return gpt4_response

        except Exception as e:
            self.get_logger().error(f"Error calling GPT-4 API: {str(e)}")
            return "Sorry, I couldn't process your request due to an error."

    def play_response(self, response):
        try:
            # Use the TTS engine to say the response out loud
            self.tts_engine.say(response)
            self.tts_engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f"Error playing response through speaker: {str(e)}")

    def execute_robot_command(self, response):
        # Convert the response to lowercase to handle case-insensitivity
        response = response.data.lower()
        action_list = response.split(",")
        # TODO: Implement the robot command execution logic, in a large if-else statement. Your conditionals should be set based on the expected commands from GPT-4, and the corresponding methods should be called on the KarelPupper object.
        for action in action_list:

            if "move" in action:
                self.pupper.move()
            elif "left" in action:
                self.pupper.turn_left()
            elif "right" in action:
                self.pupper.turn_right()
            elif "bark" in action:
                self.pupper.bark()
            else:
                pass # Do nothing for inproper action


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
