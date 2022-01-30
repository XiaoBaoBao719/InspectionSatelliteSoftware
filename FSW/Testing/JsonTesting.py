import json

tester_string = '{"name": "Nitin", "dept": "False", "company": "GFG"}'
                    
test_dict = json.loads(tester_string)

print(test_dict)
print(test_dict['dept'])

print("Datatype")
print(type(test_dict))

with open("test_json.json", "w") as out:
    json.dump(test_dict, out)
    
f= open('test_json.json',)

data = json.load(f)

for i in data:
    print(i)
    
f.close()