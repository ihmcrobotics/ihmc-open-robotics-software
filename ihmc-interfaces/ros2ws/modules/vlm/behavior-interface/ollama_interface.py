import os
import json
from pathlib import Path
from datetime import datetime
import ollama


class OllamaInterface:
    """Drop-in replacement for VLMInterface that uses a local Ollama server.

    Accepts the same JSON config files as VLMInterface. The only config fields
    that change meaning:
      - api_base  → Ollama host URL, e.g. "http://localhost:11434"
                    (omit the /v1 suffix used by the OpenAI-compatible endpoint)
      - model     → Ollama model tag, e.g. "llava", "qwen2.5vl", "llama3.2-vision"
                    Use "auto" to pick the first model returned by `ollama list`.
    """

    def __init__(self, config_file):
        self.load_config(config_file)
        host = self.api_base.rstrip("/")
        self.client = ollama.Client(host=host)
        if self.model == "auto":
            models = self.client.list().models
            if not models:
                raise RuntimeError("No models found in Ollama. Pull one first (e.g. ollama pull llava).")
            self.model = models[0].model
        self.log_file = "vlm_logs.json"

    # ------------------------------------------------------------------
    # Config loading  (identical to VLMInterface)
    # ------------------------------------------------------------------

    def load_config(self, config_file):
        with open(config_file, "r") as f:
            config = json.load(f)

        self.api_base        = config.get("ollama_host", "http://localhost:11434")
        self.model           = config.get("model", "auto")
        self.temperature     = config.get("temperature", 0.0)
        self.max_tokens      = config.get("max_tokens", 512)
        self.top_p           = config.get("top_p", 1.0)
        self.supports_vision = config.get("supports_vision", False)
        self.image           = config.get("image", None)
        self.prompt_type     = config.get("prompt_type", "vlm_prompt")
        self.description     = config.get("description", "")

        self.system_prompt    = self._join(config.get("system_prompt", ""))
        self.task_description = self._join(config.get("task_description", ""))
        self.examples         = self._join(config.get("examples", ""))
        self.immediate_task   = config.get("immediate_task", "")
        self.precognition     = config.get("precognition", "")
        self.output_format    = self._join(config.get("output_format", ""))

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
        if isinstance(value, list):
            return "\n".join(value)
        return value or ""

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def call_model(self, vlm_input, image_path=None):
        """
        Query the model via Ollama.

        Args:
            vlm_input:   Runtime context string (scene objects, available behaviors, etc.)
            image_path:  Path to an image file. Overrides the config 'image' field.
                         Pass None to use the config default; pass "" to force text-only.
        """
        self.vlm_input = vlm_input

        img = image_path if image_path is not None else self.image
        user_text = vlm_input + self.prompt

        # Ollama native API passes images as raw bytes in a list
        images = [Path(img).read_bytes()] if img and self.supports_vision else []

        messages = [
            {"role": "system", "content": self.system_prompt},
            {"role": "user",   "content": user_text, **({"images": images} if images else {})},
        ]

        response = self.client.chat(
            model=self.model,
            messages=messages,
            options={
                "temperature": self.temperature,
                "top_p":       self.top_p,
                "num_predict": self.max_tokens,
            },
        )

        output = response.message.content
        self.log_interaction(output)
        return output

    # ------------------------------------------------------------------
    # Logging  (identical to VLMInterface)
    # ------------------------------------------------------------------

    def first_log_interaction(self, vlm_input):
        """Log model/config metadata on the first call (call before call_model)."""
        log_data = {
            "timestamp":       datetime.now().isoformat(),
            "model":           self.model,
            "ollama_host":     self.api_base,
            "temperature":     self.temperature,
            "max_tokens":      self.max_tokens,
            "top_p":           self.top_p,
            "supports_vision": self.supports_vision,
            "description":     self.description,
            "prompt_type":     self.prompt_type,
            "system_prompt":   self.system_prompt,
            "prompt":          self.prompt,
            "input":           vlm_input,
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
