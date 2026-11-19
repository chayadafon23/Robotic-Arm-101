# Hugging Face Space entrypoint for demo/; this is not the LeRobot EnvHub server image.
FROM python:3.12-slim-bookworm
WORKDIR /app
COPY demo/requirements.txt /tmp/demo-requirements.txt
RUN pip install --no-cache-dir -r /tmp/demo-requirements.txt \
    && useradd -m -u 1000 user
COPY --chown=user:user . .
USER user
ENV MUJOCO_GL=disable OPENBLAS_NUM_THREADS=1 PYTHONUNBUFFERED=1
EXPOSE 7860
CMD ["python", "-m", "demo.app"]
