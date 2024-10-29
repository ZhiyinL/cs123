import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
from openai import OpenAI

client = OpenAI(api_key='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')

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

    def query_callback(self, msg):
        user_query = msg.data
        self.get_logger().info(f"Received user query: {user_query}")

        # Call GPT-4 API to get the response
        response = self.get_gpt4_response(user_query)

        # Publish the response to the ROS2 topic
        response_msg = String()
        response_msg.data = response
        self.publisher_.publish(response_msg)
        self.get_logger().info(f"Published GPT-4 response: {response}")

        # Play the response through the speaker
        self.play_response(response)

    def get_gpt4_response(self, query):
        try:
            # Making the API call to GPT-4 using OpenAI's Python client
            response = client.chat.completions.create(model="gpt-4",  # Model identifier, assuming GPT-4 is used
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
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
