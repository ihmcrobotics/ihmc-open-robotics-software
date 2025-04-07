# LLM API imports
from together import Together
import os
import requests
import json
from datetime import datetime

# Get a free API key from https://api.together.xyz/settings/api-keys

def llama32(messages, model_size=3):
  model = f"meta-llama/Llama-3.2-{model_size}B-Instruct-Turbo"
  url = "https://api.together.xyz/v1/chat/completions"
  payload = {
    "model": model,
    "max_tokens": 4096,
    "temperature": 0.0,
    "stop": ["<|eot_id|>","<|eom_id|>"],
    "messages": messages
  }

  headers = {
    "Accept": "application/json",
    "Content-Type": "application/json",
    "Authorization": "Bearer " + os.environ["TOGETHER_API_KEY"]
  }
  res = json.loads(requests.request("POST", url, headers=headers, data=json.dumps(payload)).content)

  if 'error' in res:
    raise Exception(res['error'])

  return res['choices'][0]['message']['content']


# Create a LLM interface class
class LLMInterface:
    def __init__(self, config_file):
        self.load_config(config_file)
        self.api_url = "https://api.together.xyz/v1/chat/completions"
        self.api_key = os.environ.get("TOGETHER_API_KEY")
        self.log_file = "llm_logs.json"

    def load_config(self, config_file):
        with open(config_file, "r") as file:
            config = json.load(file)
        
        self.model = config.get("model", "meta-llama/Llama-3.2-3B-Instruct-Turbo")
        self.temperature = config.get("temperature", 0.0)
        self.token_limit = config.get("token_limit", 4096)
        self.top_k = config.get("top_k", "N/A")
        self.top_p = config.get("top_p", 1.0)
        self.goal = config.get("goal", "")
        self.prompt = config.get("prompt", "Summarize all the behaviors in one paragraph")

    def call_model(self, llm_input):
        #messages = [{"role": "user", "content": self.prompt}]
        messages = [
        {
           "role": "system",
           "content": """ You are an AI reasoning module for a robot operating in a dynamic environment. Your task is to decide the most appropriate action the robot should take next based on:

                            Scene Objects: The objects currently detected in the environment and their relationships.

                            Available Behaviors: A list of actions the robot can perform.

                            Task Description: The high-level goal the robot needs to accomplish."""
        },
        {
           "role": "assistant",
           "content": llm_input
        },
        {
           "role": "user",
           "content": self.prompt
        }
        ]
        
        payload = {
            "model": self.model,
            "max_tokens": self.token_limit,
            "temperature": self.temperature,
            "top_k": self.top_k,
            "top_p": self.top_p,
            "messages": messages
        }

        headers = {
            "Accept": "application/json",
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }

        response = requests.post(self.api_url, headers=headers, json=payload)
        res_json = response.json()
        
        if 'error' in res_json:
            raise Exception(res_json['error'])
        
        output = res_json['choices'][0]['message']['content']
        self.log_interaction(messages)
        self.log_interaction(output)
        
        return output

    def log_interaction(self, output):
        log_data = {
            "timestamp": datetime.now().isoformat(),
            "model": self.model,
            "temperature": self.temperature,
            "token_limit": self.token_limit,
            "top_k": self.top_k,
            "top_p": self.top_p,
            "goal": self.goal,
            "prompt": self.prompt,
            "output": output
        }
        
        try:
            if os.path.exists(self.log_file):
                with open(self.log_file, "r") as file:
                    logs = json.load(file)
            else:
                logs = []
        except json.JSONDecodeError:
            logs = []
        
        logs.append(log_data)
        
        with open(self.log_file, "w") as file:
            json.dump(logs, file, indent=4)

# Example usage:
# print(" Calling the LLM")
# llm = LLMInterface(config_file="config.json")
# response = llm.call_model(behaviors_str)
# print(response)


    #     # Get available behaviors once at initialization
    #     behavior_queue = msg.available_behaviors
    #     print("Available behaviors:", behavior_queue)

    #     if not behavior_queue:
    #         print("[ERROR] No available behaviors.")
    #         return

    #     initialized = True
    #     current_behavior_index = 0  # Start from the first behavior

    # # Monitor behavior execution
    # completed_behavior = msg.completed_behavior
    # if completed_behavior and completed_behavior in behavior_queue:
    #     print(f"Completed Behavior: {completed_behavior}")
    #     waiting_for_command = True
    #     current_behavior_index += 1  # Move to the next behavior

    # if current_behavior_index < len(behavior_queue) and waiting_for_command:
    #     # Execute the next behavior in the list
    #     behavior_command = AI2RCommandMessage()
    #     behavior_command.behavior_to_execute = behavior_queue[current_behavior_index]
    #     print(f"Commanded Behavior: {behavior_command.behavior_to_execute}")
    #     ros2["behavior_publisher"].publish(behavior_command)
        