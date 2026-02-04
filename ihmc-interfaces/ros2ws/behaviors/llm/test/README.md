# How to write the config.json file

######################################## INPUT VARIABLES ########################################

# First input variable - the conversation history (this can also be added as preceding `user` and `assistant` messages in the API call)
HISTORY = """Customer: Give me two possible careers for sociology majors.

Joe: Here are two potential careers for sociology majors:

- Social worker - Sociology provides a strong foundation for understanding human behavior and social systems. With additional training or certification, a sociology degree can qualify graduates for roles as social workers, case managers, counselors, and community organizers helping individuals and groups.

- Human resources specialist - An understanding of group dynamics and organizational behavior from sociology is applicable to careers in human resources. Graduates may find roles in recruiting, employee relations, training and development, diversity and inclusion, and other HR functions. The focus on social structures and institutions also supports related careers in public policy, nonprofit management, and education."""

# Second input variable - the user's question
QUESTION = "Which of the two careers requires more than a Bachelor's degree?"



######################################## PROMPT ELEMENTS ########################################

##### Prompt element 1: `user` role
# Make sure that your Messages API call always starts with a `user` role in the messages array.
# The get_completion() function as defined above will automatically do this for you.

##### Prompt element 2: Task context
# Give Claude context about the role it should take on or what goals and overarching tasks you want it to undertake with the prompt.
# It's best to put context early in the body of the prompt.
TASK_CONTEXT = "You will be acting as an AI career coach named Joe created by the company AdAstra Careers. Your goal is to give career advice to users. You will be replying to users who are on the AdAstra site and who will be confused if you don't respond in the character of Joe."

##### Prompt element 3: Tone context
# If important to the interaction, tell Claude what tone it should use.
# This element may not be necessary depending on the task.
TONE_CONTEXT = "You should maintain a friendly customer service tone."

##### Prompt element 4: Detailed task description and rules
# Expand on the specific tasks you want Claude to do, as well as any rules that Claude might have to follow.
# This is also where you can give Claude an "out" if it doesn't have an answer or doesn't know.
# It's ideal to show this description and rules to a friend to make sure it is laid out logically and that any ambiguous words are clearly defined.
TASK_DESCRIPTION = """Here are some important rules for the interaction:
- Always stay in character, as Joe, an AI from AdAstra Careers
- If you are unsure how to respond, say \"Sorry, I didn't understand that. Could you rephrase your question?\"
- If someone asks something irrelevant, say, \"Sorry, I am Joe and I give career advice. Do you have a career question today I can help you with?\""""

##### Prompt element 5: Examples
# Provide Claude with at least one example of an ideal response that it can emulate. Encase this in  XML tags. Feel free to provide multiple examples.
# If you do provide multiple examples, give Claude context about what it is an example of, and enclose each example in its own set of XML tags.
# Examples are probably the single most effective tool in knowledge work for getting Claude to behave as desired.
# Make sure to give Claude examples of common edge cases. If your prompt uses a scratchpad, it's effective to give examples of how the scratchpad should look.
# Generally more examples = better.
EXAMPLES = """Here is an example of how to respond in a standard interaction:

Customer: Hi, how were you created and what do you do?
Joe: Hello! My name is Joe, and I was created by AdAstra Careers to give career advice. What can I help you with today?
"""

##### Prompt element 6: Input data to process
# If there is data that Claude needs to process within the prompt, include it here within relevant XML tags.
# Feel free to include multiple pieces of data, but be sure to enclose each in its own set of XML tags.
# This element may not be necessary depending on task. Ordering is also flexible.
INPUT_DATA = f"""Here is the conversational history (between the user and you) prior to the question. It could be empty if there is no history:

{HISTORY}


Here is the user's question:

{QUESTION}
"""

##### Prompt element 7: Immediate task description or request #####
# "Remind" Claude or tell Claude exactly what it's expected to immediately do to fulfill the prompt's task.
# This is also where you would put in additional variables like the user's question.
# It generally doesn't hurt to reiterate to Claude its immediate task. It's best to do this toward the end of a long prompt.
# This will yield better results than putting this at the beginning.
# It is also generally good practice to put the user's query close to the bottom of the prompt.
IMMEDIATE_TASK = "How do you respond to the user's question?"

##### Prompt element 8: Precognition (thinking step by step)
# For tasks with multiple steps, it's good to tell Claude to think step by step before giving an answer
# Sometimes, you might have to even say "Before you give your answer..." just to make sure Claude does this first.
# Not necessary with all prompts, though if included, it's best to do this toward the end of a long prompt and right after the final immediate task request or description.
PRECOGNITION = "Think about your answer first before you respond."

##### Prompt element 9: Output formatting
# If there is a specific way you want Claude's response formatted, clearly tell Claude what that format is.
# This element may not be necessary depending on the task.
# If you include it, putting it toward the end of the prompt is better than at the beginning.
OUTPUT_FORMATTING = "Put your response in  tags."

##### Prompt element 10: Prefilling Claude's response (if any)
# A space to start off Claude's answer with some prefilled words to steer Claude's behavior or response.
# If you want to prefill Claude's response, you must put this in the `assistant` role in the API call.
# This element may not be necessary depending on the task.
PREFILL = "[Joe] "



######################################## COMBINE ELEMENTS ########################################

PROMPT = ""

if TASK_CONTEXT:
    PROMPT += f"""{TASK_CONTEXT}"""

if TONE_CONTEXT:
    PROMPT += f"""\n\n{TONE_CONTEXT}"""

if TASK_DESCRIPTION:
    PROMPT += f"""\n\n{TASK_DESCRIPTION}"""

if EXAMPLES:
    PROMPT += f"""\n\n{EXAMPLES}"""

if INPUT_DATA:
    PROMPT += f"""\n\n{INPUT_DATA}"""

if IMMEDIATE_TASK:
    PROMPT += f"""\n\n{IMMEDIATE_TASK}"""

if PRECOGNITION:
    PROMPT += f"""\n\n{PRECOGNITION}"""

if OUTPUT_FORMATTING:
    PROMPT += f"""\n\n{OUTPUT_FORMATTING}"""

# Print full prompt
print("--------------------------- Full prompt with variable substutions ---------------------------")
print("USER TURN")
print(PROMPT)
print("\nASSISTANT TURN")
print(PREFILL)

source /opt/ros/humble/setup.bash    
./compile_interfaces.bash 
source install/setup.bash
export ROS_DOMAIN_ID=128