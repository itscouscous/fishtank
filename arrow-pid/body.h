const char body[] = R"=====( 
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Arrow Key Controller</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            text-align: center;
            user-select: none;
            -webkit-user-select: none;
            touch-action: manipulation;
        }
        .container {
            max-width: 600px;
            margin: 0 auto;
        }
        .status {
            margin: 20px 0;
            padding: 10px;
            background-color: #f0f0f0;
            border-radius: 5px;
            font-weight: bold;
        }
        .control-buttons {
            display: grid;
            grid-template-columns: repeat(3, 60px);
            grid-template-rows: repeat(3, 60px);
            gap: 5px;
            justify-content: center;
            margin: 20px 0;
        }
        .btn {
            background-color: #e0e0e0;
            border: 1px solid #999;
            border-radius: 5px;
            font-size: 20px;
            display: flex;
            align-items: center;
            justify-content: center;
            cursor: pointer;
            transition: background-color 0.2s;
        }
        .btn:active, .btn.active {
            background-color: #007bff;
            color: white;
        }
        #btn-up {
            grid-column: 2;
            grid-row: 1;
        }
        #btn-left {
            grid-column: 1;
            grid-row: 2;
        }
        #btn-right {
            grid-column: 3;
            grid-row: 2;
        }
        #btn-down {
            grid-column: 2;
            grid-row: 3;
        }
        #response {
            margin-top: 20px;
            color: #666;
            min-height: 20px;
        }
        .instruction {
            margin-top: 30px;
            font-size: 14px;
            color: #666;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Arrow Key Controller</h1>
        
        <div class="status">
            X-Axis: <span id="x-axis">0</span>, 
            Y-Axis: <span id="y-axis">0</span>
        </div>
        
        <!-- Touch buttons for mobile users -->
        <div class="control-buttons">
            <div class="btn" id="btn-up">↑</div>
            <div class="btn" id="btn-left">←</div>
            <div class="btn" id="btn-right">→</div>
            <div class="btn" id="btn-down">↓</div>
        </div>
        
        <div id="response">Ready</div>
        
        <div class="instruction">
            <p>Use arrow keys on keyboard or touch buttons above to control.</p>
        </div>
    </div>

    <script>
        // Current state
        const state = {
            x: 0,  // -1: left, 0: center, 1: right
            y: 0   // -1: down, 0: center, 1: up
        };
        
        // Track which keys are currently pressed
        const keysPressed = {
            ArrowUp: false,
            ArrowDown: false,
            ArrowLeft: false,
            ArrowRight: false
        };
        
        // DOM elements
        const xAxisDisplay = document.getElementById('x-axis');
        const yAxisDisplay = document.getElementById('y-axis');
        const responseDisplay = document.getElementById('response');
        const touchButtons = {
            ArrowUp: document.getElementById('btn-up'),
            ArrowDown: document.getElementById('btn-down'),
            ArrowLeft: document.getElementById('btn-left'),
            ArrowRight: document.getElementById('btn-right')
        };
        
        // Function to update state based on currently pressed keys
        function updateState() {
            const previousX = state.x;
            const previousY = state.y;
            
            // Determine X axis state
            if (keysPressed.ArrowLeft && !keysPressed.ArrowRight) {
                state.x = -1;
            } else if (!keysPressed.ArrowLeft && keysPressed.ArrowRight) {
                state.x = 1;
            } else {
                state.x = 0;
            }
            
            // Determine Y axis state
            if (keysPressed.ArrowUp && !keysPressed.ArrowDown) {
                state.y = 1;
            } else if (!keysPressed.ArrowUp && keysPressed.ArrowDown) {
                state.y = -1;
            } else {
                state.y = 0;
            }
            
            // Update display
            xAxisDisplay.textContent = state.x;
            yAxisDisplay.textContent = state.y;
            
            // Update button visual state
            updateButtonVisuals();
            
            // Only send data when state changes
            if (previousX !== state.x || previousY !== state.y) {
                sendToESP(state.x, state.y);
            }
        }
        
        // Function to update button visual state
        function updateButtonVisuals() {
            for (const [key, element] of Object.entries(touchButtons)) {
                if (keysPressed[key]) {
                    element.classList.add('active');
                } else {
                    element.classList.remove('active');
                }
            }
        }
        
        // Function to send data to ESP
        function sendToESP(x, y) {
            const command = `x:${x},y:${y}`;
            const xhr = new XMLHttpRequest();
            xhr.open("GET", "/cmd?val=" + command, true);
            xhr.onreadystatechange = function() {
                if (xhr.readyState === 4) {
                    if (xhr.status === 200) {
                        responseDisplay.textContent = xhr.responseText;
                    } else {
                        responseDisplay.textContent = "Error communicating with ESP";
                    }
                }
            };
            xhr.send();
        }
        
        // Keyboard event listeners
        window.addEventListener('keydown', (event) => {
            if (keysPressed.hasOwnProperty(event.key) && !keysPressed[event.key]) {
                keysPressed[event.key] = true;
                updateState();
            }
            // Prevent scrolling with arrow keys
            if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.key)) {
                event.preventDefault();
            }
        });
        
        window.addEventListener('keyup', (event) => {
            if (keysPressed.hasOwnProperty(event.key)) {
                keysPressed[event.key] = false;
                updateState();
            }
        });
        
        // When window loses focus, reset all keys
        window.addEventListener('blur', () => {
            for (const key in keysPressed) {
                keysPressed[key] = false;
            }
            updateState();
        });
        
        // Touch events for mobile controls
        for (const [key, element] of Object.entries(touchButtons)) {
            // Touch start - simulate keydown
            element.addEventListener('touchstart', (event) => {
                event.preventDefault();
                keysPressed[key] = true;
                updateState();
            });
            
            // Touch end - simulate keyup
            element.addEventListener('touchend', (event) => {
                event.preventDefault();
                keysPressed[key] = false;
                updateState();
            });
            
            // Mouse events for desktop testing
            element.addEventListener('mousedown', (event) => {
                event.preventDefault();
                keysPressed[key] = true;
                updateState();
            });
            
            element.addEventListener('mouseup', (event) => {
                event.preventDefault();
                keysPressed[key] = false;
                updateState();
            });
            
            // Prevent context menu on long press
            element.addEventListener('contextmenu', (event) => {
                event.preventDefault();
            });
        }
    </script>
</body>
</html>
)=====";