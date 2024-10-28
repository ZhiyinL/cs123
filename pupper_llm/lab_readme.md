# Run GPT Responder

cd ~/pupper_llm/launch
ros2 launch launch_no_walk.py

Add in API Key
cd ~/pupper_llm/pupper_llm/
python3 simple_gpt_responder.py

New Terminal
ros2 topic pub /user_query_topic std_msgs/String "data: 'What is your name?'" --once
