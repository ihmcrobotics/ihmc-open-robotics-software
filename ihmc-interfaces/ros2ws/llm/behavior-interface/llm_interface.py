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
        
        self.model          = config.get("model", "meta-llama/Llama-3.2-3B-Instruct-Turbo")
        self.temperature    = config.get("temperature", 0.0)
        self.token_limit    = config.get("token_limit", 4096)
        self.top_k          = config.get("top_k", "N/A")
        self.top_p          = config.get("top_p", 1.0)
        self.goal           = config.get("goal", "")
        self.system_prompt  = config.get("system_prompt", "")
        self.prompt_type    = config.get("prompt_type", "simple_prompt")
        self.prompt         = config.get("prompt", "")
        self.task_description = config.get("task_description", "")
        self.examples       = config.get("examples", "")
        self.immediate_task = config.get("immediate_task", "")
        self.precognition = config.get("precognition", "")
        self.output_formatting = config.get("output_formatting", "")

        if self.task_description:
            self.prompt += f"\nTask Description: {self.task_description}"
        if self.examples:
            self.prompt += f"\nExamples: {self.examples}"
        if self.immediate_task:
            self.prompt += f"\nImmediate Task: {self.immediate_task}"
        if self.precognition:
            self.prompt += f"\nPrecognition: {self.precognition}"
        if self.output_formatting:
            self.prompt += f"\nOutput Formatting: {self.output_formatting}"


    def call_model(self, llm_input):
        #messages = [{"role": "user", "content": self.prompt}]
        self.llm_input = llm_input
        messages = [
        {
           "role": "system",
           "content": self.system_prompt
        },
        {
           "role": "user",
           "content": self.llm_input + " " + self.prompt
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
        #self.log_interaction(messages)
        self.log_interaction(output)
        
        return output
    
        
    def first_log_interaction(self, output):
        log_data = {
            "timestamp": datetime.now().isoformat(),
            "model": self.model,
            "temperature": self.temperature,
            "token_limit": self.token_limit,
            "top_k": self.top_k,
            "top_p": self.top_p,
            "goal": self.goal,
            "system_prompt": self.system_prompt,
            "prompt_type": self.prompt_type,
            "prompt": self.prompt,
        }
        
        # the folder to save the log file is created if it does not exist 
        # log_folder = pwd + model + prompt_type
        log_path = os.path.join(os.getcwd(), self.model, self.prompt_type, self.log_file)

        # Ensure the directory exists
        os.makedirs(os.path.dirname(log_path), exist_ok=True)

        # If the file doesn't exist, create it with initial log_data
        if not os.path.exists(log_path):
            with open(log_path, "w") as f:
                json.dump([log_data], f, indent=4)
        else:
            with open(log_path, "r+") as f:
                try:
                    data = json.load(f)
                except json.JSONDecodeError:
                    data = []

                data.append(log_data)
                f.seek(0)
                json.dump(data, f, indent=4)
                f.truncate()


    def log_interaction(self, output):
        log_data = {
            "prompt": self.llm_input + " " + self.prompt,
            "output": output
        }
        
        # the folder to save the log file is created if it does not exist 
        # log_folder = pwd + model + prompt_type
        log_path = os.path.join(os.getcwd(), self.model, self.prompt_type, self.log_file)

        # Ensure the directory exists
        os.makedirs(os.path.dirname(log_path), exist_ok=True)

        # If the file doesn't exist, create it with initial log_data
        if not os.path.exists(log_path):
            with open(log_path, "w") as f:
                json.dump([log_data], f, indent=4)
        else:
            with open(log_path, "r+") as f:
                try:
                    data = json.load(f)
                except json.JSONDecodeError:
                    data = []

                data.append(log_data)
                f.seek(0)
                json.dump(data, f, indent=4)
                f.truncate()

