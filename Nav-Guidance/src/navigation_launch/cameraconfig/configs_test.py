import json
with open ("configs.json") as configfile:
  data = "".join(configfile.readlines()).replace('\n','')
print(data)
print(json.loads(data))
