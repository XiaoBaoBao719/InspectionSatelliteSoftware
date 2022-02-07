import json

tester_string = '{"Bootcounter": "Nitin", "dept": "merch", "company": "GFG"}'
                    
j_object = json.loads(tester_string)

# print(test_dict)
# #print(test_dict['dept'])

# # print("Datatype")
# # print(type(test_dict))

# with open("test_json.json", "w") as out:
#      json.dump(test_dict, out)

#data = json.load(f)

# for i in data:
#     print(i)
    
#f.close()

filename = "test_json.json"
new_msg = "hello"

with open(filename, "w") as output_json:
    output_string = json.dump(output_json, j_object)
    print(output_string)

# with open(filename, 'r') as f:
#     string = json.dumps(f)
#     print(string)

# with open(filename, 'w') as f:
#     f.write(message)

# # with open(filename, 'w') as f:
# #     json.dump(data, f)

# foo = open(filename, 'w')
# data = json.load(foo)

# for i in foo:
#     print(i)

output_json.close()