import json
some_string = '{"name" : "john", "status" : "False", "bootctr" : 0}'

y = json.loads(some_string) # allows you to parse json string
#print(y["bootctr"])
#print(type(y["bootctr"]))

# python object
x = {"name":"sam", "age":30, "city":"new york"}

json_string = json.dumps(x) # converts from python object into json string

#parse the string
print(json_string)

with open("test_json.json", 'w') as json_out:
    json_out.write(json_string)

#temp = '{"":""}'
temp = {}
with open("test_json.json", 'r') as json_out:
    #print(json_out.read()) # read outputs a string
    temp = json.loads(json_out.read())
    #print(type(temp))
    print(temp)
    print(temp["name"])
    #print(type(json_out))

#temp["name"] = "ham"
#json_string = json.dumps(temp)

# with open("test_json.json", 'w') as json_out:
#     json_out.write(json_string)
#     print("WRITING")

with open("test_json.json", 'r') as json_out:
    new_msg = json.loads(json_out.read())
    #print(type(new_msg["name"]))
    new_msg.update(name="ham")
    temp = new_msg

with open("test_json.json", 'w') as json_out:
    json_out.write(json.dumps(temp))

with open("test_json.json", 'r') as json_out:
    #print(json_out.read())
    out = json.loads(json_out.read())
    print(out["name"])
    pass