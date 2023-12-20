import json
import json

# print json
# with open('LLM_env/Assets/LLM_ENV/json_data/replan0_call0_agentAlice_1123-1656.json', 'r') as file:
#     data = json.load(file)
#     message = data[0].get('message')
#     print(message)

    # Read text file
with open('LLM_env/Assets/LLM_ENV/json_data/json.txt', 'r') as file:
    text_content = file.read()

    # Convert text to JSON
    json_data = {'content': text_content}

    # Write JSON to file
with open('LLM_env/Assets/LLM_ENV/json_data/test.json', 'w') as file:
    json.dump(json_data, file)
