const io = require('socket.io')({
    cors: {
        origin: ['http://localhost:3000']
    }
});

var servoExamples = [
    
    {"id": "servo_1", "name": "Servo 1", "stateID": false, "temperature": 0, "current": 0, "voltage": 0, "angle": 0, "currentSpeed": 0},
    {"id": "servo_2", "name": "Servo 2", "stateID": false, "temperature": 0, "current": 0, "voltage": 0, "angle": 0, "currentSpeed": 0},
    

]; 

io.on('connection', socket => {
    socket.on('catalogueShift', (catalogue) => {
        servoExamples = catalogue;
    });
    socket.on('updateAngle', (givenID, goalAngle) => {
        const index = servoExamples.findIndex((item) => { return item.id === givenID; });
        console.log(`Index is ${index}, ID is ${givenID}, and goalAngle is ${goalAngle}`)
        servoExamples[index].angle = goalAngle;
    });
    socket.on('updateServoExample', (servoData) => {
        servoExamples.push(servoData);
    });
});

io.listen(3001);

setInterval(() => {
    io.emit('message', servoExamples);
}, 5000);
