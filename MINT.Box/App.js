// Code by Taksch A. Dube for Capstone Project Spring 2021

import './App.css';
import { DragDropContext, Droppable, Draggable } from 'react-beautiful-dnd'; //first drag lib
import React, { useState } from 'react'
import Draggable2 from 'react-draggable';

// servo attributes: units
// id: string, name: string, stateID: string, state_color: string, temperature: int(C),
// current: int(mA), voltage: int(V), angle: int(deg), goalSpeed: int(rpm), currentSpeed: int(rpm).
let servoExamples = [ // This is here for testing. Static catalogue-elements
    {
        id: 'servo_1',
        name: 'Servo 1',
        stateID: 'on',
        state_color: '#4e823e',
        temperature: '0C',
        current: '0mA',
        voltage: '0V',
        angle: '100', 
        goalPosition: '0',
        currentSpeed: '0rpm',
    },
    {
        id: 'servo_2',
        name: 'Servo 2',
        stateID: 'off',
        state_color: '#f54c4c',
        temperature: '0C',
        current: '0mA',
        voltage: '0V',
        angle: '66', 
        goalPosition: '0',
        currentSpeed: '0rpm',
    },
    {
        id: 'servo_3',
        name: 'Servo 3',
        stateID: 'off',
        state_color:'#f54c4c',
        temperature: '0C',
        current: '0mA',
        voltage: '0V',
        angle: '50', 
        goalPosition: '0',
        currentSpeed: '0rpm',
    },
    {
        id: 'servo_4',
        name: 'Servo 4',
        stateID: 'on',
        state_color:'#4e823e',
        temperature: '0C',
        current: '0mA',
        voltage: '0V',
        angle: '0', 
        goalPosition: '0',
        currentSpeed: '0rpm',
    },
    {
        id: 'servo_5',
        name: 'Servo 5',
        stateID: 'off',
        state_color:'#f54c4c',
        temperature: '0C',
        current: '0mA',
        voltage: '0V',
        angle: '59', 
        goalPosition: '0',
        currentSpeed: '0rpm',
    },
    {
        id: 'servo_6',
        name: 'Servo 6',
        stateID: 'on',
        state_color:'#4e823e',
        temperature: '0C',
        current: '0mA',
        voltage: '0V',
        angle: '350', 
        goalPosition: '0',
        currentSpeed: '0rpm',
    },
    {
        id: 'servo_7',
        name: 'Servo 7',
        stateID: 'on',
        state_color:'#4e823e',
        temperature: '0C',
        current: '0mA',
        voltage: '0V',
        angle: '2', 
        goalPosition: '0',
        currentSpeed: '0rpm',
    },
    {
        id: 'servo_8',
        name: 'Servo 8',
        stateID: 'off',
        state_color:'#eb9731',
        temperature: '0C',
        current: '0mA',
        voltage: '0V',
        angle: '0', 
        goalPosition: '0',
        currentSpeed: '0rpm',
    },
    {
        id: 'servo_9',
        name: 'Servo 9',
        stateID: 'off',
        state_color: '#eb9731',
        temperature: '0C',
        current: '0mA',
        voltage: '0V',
        angle: '270', 
        goalPosition: '0',
        currentSpeed: '0rpm',
    },
];

function Catalogue() { // React Component sans parent

    const [catalogue, updateCatalogue] = useState(servoExamples);
    //updateCatalogue will be modified below

    //this function is to update the list when catalogue-elements are moved
    function handleOnDragEnd(result) {
        if (!result.destination) return;
        const items = Array.from(catalogue);
        const [reorderedItems] = items.splice(result.source.index, 1);
        items.splice(result.destination.index, 0, reorderedItems);

        updateCatalogue(items); //store into updateCatalogue
    }

    //This return function is for rendering the divs. It's all front-end so there is nothing
    //related to the library here.
    return (
        <div className='catalogue'>
            <DragDropContext onDragEnd={handleOnDragEnd}>
                <Droppable droppableId='catalogue'>
                    {(provided) => (
                        <ul className="catalogueElements" {...provided.droppableProps}
                            ref={provided.innerRef}>
                            {catalogue.map(({ id, name, stateID, state_color }, index) => {
                                return (
                                    <Draggable key={id} draggableId={id} index={index}>
                                        {(provided) => (
                                            <li ref={provided.innerRef} {...provided.draggableProps}
                                                {...provided.dragHandleProps}>
                                                <div className="catalogueElement"
                                                    style={{backgroundColor: `${state_color}`}}>{name}</div>
                                            </li> // This is the catalogue entries 
                                            // that are seen and are draggable 
                                            // within the catalogue.
                                        )}
                                    </Draggable>
                                )
                            })}
                            {provided.placeholder}
                        </ul>

                    )}
                </Droppable>
                <button className="circleAddButton" 
                onClick={() => alert(JSON.stringify(servoExamples)) /* Update the list of servos
                                                Refresh the entire app */}>
                    +
                          </button>
            </DragDropContext>
        </div>
    );
}

//Explaining the canvas structure
// Canvas -> CanvasElement -> TitleBar, Angle Tracker -> Angle Pointer -> Start Button, 
// CursoryInfoBar

function Canvas() { // The canvas element lol
    const [canvas] = useState(servoExamples);

    return (
        <div className="canvas" id="canvasbounds"> 
            {canvas.map(({ id, name, stateID, state_color, temperature, 
            current, voltage, angle, currentSpeed, goalPosition}, index) => {
                return (
                    <Draggable2 key={id} index={index}> 
                        <div className="canvasElement">
                            <div className="canvasElementTitleBar"> 
                                {name} 
                            </div>
                            <div className="canvasElementAngleTracker">
                                <div className="canvasPointer" 
                                style={{
                                    transform: `rotate(${angle}deg)`
                                    }}> 
                                    <button className="onOffAngleButton"
                                    style={{
                                        backgroundColor: `${state_color}`, 
                                        transform: `rotate(-${angle}deg)`
                                    }} // onOffButton needs to be transformed equally
                                       // opposite the anglePointer
                                    onClick={()=>{alert(`${name} was moved with goal position at ${goalPosition}`);}}> 
                                        {angle} 
                                    </button>
                                </div>
                            </div>
                            <div className="canvasElementCursoryElementTracker">
                                {temperature} {current} {voltage} {currentSpeed} <form className="goalPositionInputForm">
                                        <input className="goalPositionInputValue" 
                                        type="text" id={`goalPositionInputValue${id}`} 
                                        onChange={()=>{goalPosition = Number(document.getElementById(`goalPositionInputValue${id}`).value)}}>
                                        </input> 
                                    </form>
                            </div>
                        </div>
                    </Draggable2>
                );
            })}

        </div>
    );
}

function App() {
    return (
        <div className="appScreen">
            <Catalogue />
            <Canvas />
        </div> // This is the entire application, tucked neatly into a single file, for better or
        // worse.
    );
}

export default App;
