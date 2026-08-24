# llm_model

Implements a non-blocking ROS 2 bridge to the OpenAI Responses API. It queues input from `/llm/input_text`, replays conversation items locally, forwards allow-listed function calls through `/llm/function_call`, submits the returned `function_call_output`, and publishes final text on `/llm/feedback`.

Set `OPENAI_API_KEY` before starting the node. See the repository root `README.md` for configuration and testing details.
