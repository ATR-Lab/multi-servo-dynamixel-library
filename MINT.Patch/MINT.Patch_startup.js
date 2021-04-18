
var python_script = 'initialize_ports.py';
const {PythonShell} = require('python-shell');
var pyshell= new PythonShell(python_script);

pyshell.on('message', function(message){
    console.log(message)
});

//pyshell.end();
var python_script2 = 'MINT.Patch_process_commands.py';
var pyshell2= new PythonShell(python_script2);

pyshell2.send(JSON.stringify("scan"))
pyshell2.send(JSON.stringify("move port_1_001 3"))
pyshell2.send(JSON.stringify("end"))

pyshell2.on('message', function(message){
    console.log(message)
    //OOGABOOGA=OOGABOOGA+'\n'+message
});

pyshell2.end()

OOGABOOGA=''

var http = require('http');
http.createServer(function (req, res) {

    res.writeHead(200, {'Content-Type': 'text/plain'});
    res.write(OOGABOOGA);
    res.end();
}).listen(8080);

//Whatever code you want between them

var python_script3 = 'MINT.Patch_constant_update.py';
var pyshell3= new PythonShell(python_script3);

pyshell3.on('message', function(message){
    //console.log(message)
    OOGABOOGA=message;
    
});

pyshell3.end();