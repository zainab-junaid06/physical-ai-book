#  ROS 2 Services

In ROS 2:

- **Topics** are for **continuous data streaming** (like sensor readings or robot state). 📡  
- **Services** are for **request/response actions** — like asking the robot to do something and waiting for confirmation. 📨

---

## 🛠️ Examples of Services

- 💡 `"Turn on the LED"`  
- 🛣️ `"Compute a path"`  
- 🤖 `"Move the arm to X, Y, Z"`  

Services work like a **function call over the network**:

client.send_request(request)
response = server.handle_request(request)

yaml
Copy code

---

## ⚡ How Services Work

1. **Server**  
   - Provides the service.  
   - Waits for requests from clients.  
   - Example: ArmController service to move a robot arm. 🕹️  

2. **Client**  
   - Calls the service when needed.  
   - Waits for a response.  
   - Example: PathPlanner client requesting a path computation. 🛣️  

---

## 📝 Service Definition

A ROS 2 service is defined in a `.srv` file with:

Request
float64 x
float64 y
float64 z
Response
bool success
string message

yaml
Copy code

- Above `---` separates **request** and **response**.  
- `success` indicates if the operation completed. ✅  
- `message` can contain extra info. ℹ️  

---

## 🔗 Key Points

- Services are **synchronous**: client waits for server response. ⏳  
- Useful for **actions that need confirmation**. ✔️  
- Often combined with **topics** for continuous monitoring. 📡  

---

ROS 2 Services let your robots **perform precise actions on demand**, making them perfect for control tasks that aren’t continuous but need guaranteed execution. 🤖⚡