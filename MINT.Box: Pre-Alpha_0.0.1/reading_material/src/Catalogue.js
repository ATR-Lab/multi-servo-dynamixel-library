import './App.css';
import { DragDropContext, Droppable, Draggable } from 'react-beautiful-dnd';
import React, { useState } from 'react'

const servoExamples = [ // This is here for testing. Static catalogue-elements
    {
        id: 'servo_1', 
        name: 'Servo 1',
        stateID: 'on',
        state_color: { backgroundColor: '#4e823e' },
    },
    {
        id: 'servo_2',
        name: 'Servo 2',
        stateID: 'idle',
        state_color: { backgroundColor: '#eb9731' },
    },
    {
        id: 'servo_3',
        name: 'Servo 3',
        stateID: 'off',
        state_color: { backgroundColor: '#f54c4c' },
    },
    {
        id: 'servo_4',
        name: 'Servo 4',
        stateID: 'on',
        state_color: { backgroundColor: '#4e823e' },
    },
    {
        id: 'servo_5',
        name: 'Servo 5',
        stateID: 'off',
        state_color: { backgroundColor: '#f54c4c' },
    },
    {
        id: 'servo_6',
        name: 'Servo 6',
        stateID: 'on',
        state_color: { backgroundColor: '#4e823e' },
    },
    {
        id: 'servo_7',
        name: 'Servo 7',
        stateID: 'on',
        state_color: { backgroundColor: '#4e823e' },
    },
    {
        id: 'servo_8',
        name: 'Servo 8',
        stateID: 'off',
        state_color: { backgroundColor: '#eb9731' },
    },
    {
        id: 'servo_9',
        name: 'Servo 9',
        stateID: 'off',
        state_color: { backgroundColor: '#eb9731' },
    },
];

function Catalogue() { // React Component sans parent

    const [catalogue, updateCatalogue] = useState(servoExamples); //updateCatalogue will be modified below

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
                        <ul className="catalogueElements" {...provided.droppableProps} ref={provided.innerRef}>
                            {catalogue.map(({ id, name, stateID, state_color }, index) => {
                                return (
                                    <Draggable key={id} draggableId={id} index={index}>
                                        {(provided) => (
                                            <li ref={provided.innerRef} {...provided.draggableProps}
                                                {...provided.dragHandleProps}>
                                                <div className="catalogueElement" style={state_color}>{name}</div>
                                            </li>
                                        )}
                                    </Draggable>
                                )
                            })}
                            {provided.placeholder}
                        </ul>
                    )}
                </Droppable>
            </DragDropContext>
        </div>
    );
}


export default Catalogue;