---
id: 1
title: Create Project Constitution
stage: constitution
date: 2025-12-17
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: hammad
command: /sp.constitution
labels: [constitution,spec-driven-development,rag-chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
 - .specify/templates/spec-template.md
 - .specify/templates/tasks-template.md
tests:
 - null
---

## Prompt

/sp.constitution

Project: Spec-Driven Technical Book with Embedded RAG Chatbot

Core principles:
- Specification-first, reproducible development
- Technical accuracy from official sources only
- Clarity for advanced developers
- End-to-end system integrity (book + chatbot)

Key standards:
- All claims traceable to authoritative documentation
- Executable, version-pinned code examples
- Spec-Kit Plus–compliant structure
- RAG chatbot must:
  - Answer from book content only
  - Cite relevant sections
  - Restrict answers to user-selected text when provided

Technology constraints:
- Book: Docusaurus → GitHub Pages
- Authoring: Claude Code + Spec-Kit Plus
- RAG stack:
  - OpenAI Agents / ChatKit SDKs
  - FastAPI
  - Neon Serverless Postgres
  - Qdrant Cloud (Free Tier)
- Chatbot embedded in published book

Quality constraints:
- No hallucinated APIs or steps
- Clear separation of content, architecture, and deployment
- Writing clarity: Flesch-Kincaid grade 11–13

Success criteria:
- Book builds and deploys successfully
- Embedded RAG chatbot functions correctly
- All specs validate and are reproducible by third parties

## Response snapshot

Created the project constitution for the "Spec-Driven Technical Book with Embedded RAG Chatbot" project, establishing core principles including specification-first development, technical accuracy from official sources, clarity for advanced developers, end-to-end system integrity, and authoritative documentation only. Updated related templates to ensure constitutional compliance across the project lifecycle.

## Outcome

- ✅ Impact: New constitution established with 5 core principles and governance structure
- 🧪 Tests: No tests required for constitution creation
- 📁 Files: Updated constitution.md and 3 template files to align with constitutional principles
- 🔁 Next prompts: Plan and implement specific features according to the new constitution
- 🧠 Reflection: Constitution provides clear framework for all future development decisions

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create feature specifications following constitutional principles