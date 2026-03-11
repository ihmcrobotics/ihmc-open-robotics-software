"""Simple client to test the Qwen2.5-7B-Instruct-AWQ vLLM server."""

from openai import OpenAI

client = OpenAI(base_url="http://localhost:8000/v1", api_key="unused")

MODEL = client.models.list().data[0].id

messages = [
    {"role": "system", "content": "You are a helpful assistant."},
    {"role": "user", "content": "What is the capital of France? Answer briefly."},
]

# Non-streaming
print("=== Non-streaming ===")
response = client.chat.completions.create(
    model=MODEL,
    messages=messages,
    max_tokens=128,
    temperature=0.7,
)
print(f"Response: {response.choices[0].message.content}")
print(f"Tokens: {response.usage.prompt_tokens} prompt + {response.usage.completion_tokens} completion")

# Streaming
print("\n=== Streaming ===")
stream = client.chat.completions.create(
    model=MODEL,
    messages=[
        {"role": "user", "content": "Write a haiku about programming."},
    ],
    max_tokens=64,
    temperature=0.7,
    stream=True,
)
for chunk in stream:
    delta = chunk.choices[0].delta.content
    if delta:
        print(delta, end="", flush=True)
print()
