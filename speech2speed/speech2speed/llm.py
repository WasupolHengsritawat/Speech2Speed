#!/usr/bin/env python3

from huggingface_hub import InferenceClient
from langchain_core.language_models import BaseChatModel
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ChatMessage
from langchain_core.outputs import ChatResult, ChatGeneration
from pydantic import PrivateAttr

class HFChatWrapper(BaseChatModel):
    # Declare private attribute for Hugging Face client
    _client: InferenceClient = PrivateAttr()

    def __init__(self, model: str, token: str):
        super().__init__()
        self._client = InferenceClient(model=model, token=token)

    @property
    def _llm_type(self) -> str:
        return "huggingface-inference-client"

    def _generate(self, messages, stop=None, run_manager=None, **kwargs) -> ChatResult:
        # Convert LangChain messages into Hugging Face format
        hf_messages = []
        for msg in messages:
            if isinstance(msg, HumanMessage):
                hf_messages.append({"role": "user", "content": msg.content})
            elif isinstance(msg, SystemMessage):
                hf_messages.append({"role": "system", "content": msg.content})
            elif isinstance(msg, AIMessage):
                hf_messages.append({"role": "assistant", "content": msg.content})
            elif isinstance(msg, ChatMessage):
                hf_messages.append({"role": msg.role, "content": msg.content})

        # Call Hugging Face chat API
        response = self._client.chat.completions.create(
            messages=hf_messages,
            max_tokens=1000,
        )
        text = response.choices[0].message.content

        return ChatResult(
            generations=[ChatGeneration(message=AIMessage(content=text))]
        )