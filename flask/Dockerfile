FROM python:3.6.4-alpine3.7

# Update
RUN apk add --update nano 

# Creating Application Source Code Directory
RUN mkdir -p /usr/src/app

# Setting Home Directory for containers
WORKDIR /usr/src/app

# Copying src code to Container
COPY . /usr/src/app

# Installing python dependencies
COPY requirements.txt /usr/src/app/
RUN pip3 install --no-cache-dir -r requirements.txt
RUN pip install gunicorn

EXPOSE  8000
CMD ["python3", "app.py"]
