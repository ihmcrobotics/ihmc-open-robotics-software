import os
import json
import base64
from pathlib import Path
from datetime import datetime
from openai import OpenAI


class VLMInterface:
    def __init__(self, config_file):
        self.load_config(config_file)
        self.client = OpenAI(base_url=self.api_base, api_key="unused")
        if self.model == "auto":
            self.model = self.client.models.list().data[0].id
        self.log_file = "vlm_logs.json"

    # ------------------------------------------------------------------
    # Config loading
    # ------------------------------------------------------------------

    def load_config(self, config_file):
        with open(config_file, "r") as f:
            config = json.load(f)

        self.api_base        = config.get("api_base", "http://localhost:8000/v1")
        self.model           = config.get("model", "auto")
        self.temperature     = config.get("temperature", 0.0)
        self.max_tokens      = config.get("max_tokens", 512)
        self.top_p           = config.get("top_p", 1.0)
        self.supports_vision = config.get("supports_vision", False)
        self.image           = config.get("image", None)   # default image path from config
        self.prompt_type     = config.get("prompt_type", "vlm_prompt")
        self.description     = config.get("description", "")

        # Array fields are joined into strings; plain strings are used as-is
        self.system_prompt    = self._join(config.get("system_prompt", ""))
        self.task_description = self._join(config.get("task_description", ""))
        self.examples         = self._join(config.get("examples", ""))
        self.immediate_task   = config.get("immediate_task", "")
        self.precognition     = config.get("precognition", "")
        self.output_format    = self._join(config.get("output_format", ""))

        # Assemble the static tail of every user message from config components
        self.prompt = ""
        if self.task_description:
            self.prompt += f"\nTask Description:\n{self.task_description}"
        if self.examples:
            self.prompt += f"\n\nExamples:\n{self.examples}"
        if self.immediate_task:
            self.prompt += f"\n\nImmediate Task: {self.immediate_task}"
        if self.precognition:
            self.prompt += f"\n\nNote: {self.precognition}"
        if self.output_format:
            self.prompt += f"\n\nOutput Format:\n{self.output_format}"

    @staticmethod
    def _join(value):
        """Join a list of strings into one string, or return the value unchanged."""
        if isinstance(value, list):
            return "\n".join(value)
        return value or ""

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def call_model(self, vlm_input, image_path=None):
        """
        Query the VLM.

        Args:
            vlm_input:   Runtime context string (scene objects, available behaviors, etc.)
            image_path:  Path to an image file. Overrides the config 'image' field.
                         Pass None to use the config default; pass "" to force text-only.
        """
        self.vlm_input = vlm_input

        # image_path=None → use config default; image_path="" → no image
        img = image_path if image_path is not None else self.image
        user_text = vlm_input + self.prompt

        if img and self.supports_vision:
            img_b64 = base64.b64encode(Path(img).read_bytes()).decode()
            user_message = {
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{img_b64}"}},
                    {"type": "text", "text": user_text},
                ],
            }
        else:
            user_message = {"role": "user", "content": user_text}

        messages = [
            {"role": "system", "content": self.system_prompt},
            user_message,
        ]

        response = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=self.temperature,
            max_tokens=self.max_tokens,
            top_p=self.top_p,
        )

        output = response.choices[0].message.content
        self.log_interaction(output)
        return output

    # ------------------------------------------------------------------
    # Logging  (mirrors llm_interface.py structure)
    # ------------------------------------------------------------------

    def first_log_interaction(self, vlm_input):
        """Log model/config metadata on the first call (call before call_model)."""
        log_data = {
            "timestamp":     datetime.now().isoformat(),
            "model":         self.model,
            "api_base":      self.api_base,
            "temperature":   self.temperature,
            "max_tokens":    self.max_tokens,
            "top_p":         self.top_p,
            "supports_vision": self.supports_vision,
            "description":   self.description,
            "prompt_type":   self.prompt_type,
            "system_prompt": self.system_prompt,
            "prompt":        self.prompt,
            "input":         vlm_input,
        }
        self._append_to_log(log_data)

    def log_interaction(self, output):
        """Log the prompt and model output after each call."""
        log_data = {
            "timestamp": datetime.now().isoformat(),
            "prompt":    self.vlm_input + self.prompt,
            "output":    output,
        }
        self._append_to_log(log_data)

    def _append_to_log(self, log_data):
        """Append a log entry to the JSON log file for this model/prompt_type."""
        # Use only the model's basename to keep the path short
        # e.g. /path/to/Qwen2.5-VL-7B-Instruct-AWQ → Qwen2.5-VL-7B-Instruct-AWQ
        model_dir = Path(self.model).name
        log_path = os.path.join(os.getcwd(), model_dir, self.prompt_type, self.log_file)
        os.makedirs(os.path.dirname(log_path), exist_ok=True)

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
