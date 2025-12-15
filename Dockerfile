FROM python:3.11-slim-bookworm

# Install i2c-tools for debugging (optional but helpful)
RUN apt-get update && apt-get install -y i2c-tools
#&& rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY app.py .

# Expose Prometheus port
EXPOSE 8000

CMD ["python", "-u", "app.py"]
