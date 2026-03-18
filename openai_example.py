"""Minimal OpenAI API demo for VS Code experiments.

Usage:
    1. pip install openai python-dotenv (optional)
    2. Create a .env file with OPENAI_API_KEY=...
    3. python openai_example.py
"""
from __future__ import annotations

import os
from openai import OpenAI

try:
    from dotenv import load_dotenv
except ImportError:
    load_dotenv = None


def get_client() -> OpenAI:
    """Create an OpenAI client using the OPENAI_API_KEY environment variable."""
    if load_dotenv is not None:
        load_dotenv()

    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError(
            "Missing OPENAI_API_KEY. Set it in the shell or create a .env file in the project root."
        )
    return OpenAI(api_key=api_key)


def run_responses_example(client: OpenAI) -> None:
    """Call the Responses API once and print the text output."""
    resp = client.responses.create(
        model="gpt-4.1-mini",
        input="Give a one-sentence summary of adaptive large neighborhood search."
    )
    print("Responses API output:\n", resp.output_text)


def run_chat_example(client: OpenAI) -> None:
    """Call the Chat Completions API for backwards compatibility."""
    resp = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": "You are a concise assistant."},
            {"role": "user", "content": "List three VRP benchmark sets."},
        ],
    )
    print("Chat Completions output:\n", resp.choices[0].message.content)


def main() -> None:
    client = get_client()
    run_responses_example(client)
    run_chat_example(client)


if __name__ == "__main__":
    main()
