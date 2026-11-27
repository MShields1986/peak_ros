#!/bin/bash
docker compose -f docker/docker-compose.yaml --profile test up --build --remove-orphans test
