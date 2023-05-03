import './App.css';

function App() {
  return (
    <div className="App">
      <header className="App-header">
      {/* From RWT HTML */}
      <div id="control-content">
        <table>
          <tr><td></td><td>startState</td><td>goalState</td></tr>
          <tr>
            <td>
              View
            </td>
            <td>
              <input type="checkbox" name="start_state" id="start_state"/>
            </td>
            <td>
              <input type="checkbox" name="goal_state" id="goal_state"/>
            </td>
          </tr>
          <tr>
            <td>
              Maniplation
            </td>
            <td>
              <input type="radio" name="manip" id="manip" value="0" checked/>
            </td>
            <td>
              <input type="radio" name="manip" id="manip" value="1"/>
            </td>
          </tr>
          </table>
        <li>IM-Size
              <input type="number" name="number" id="im-size"
                     value="0.3" min="0" max="3" step="0.1"
                     onchange="im_size_callback()"/>
        </li>
        <button id="init">Init</button>
        <button id="preview">Preview</button>
        <button id="plan">Plan</button>
        <button id="execute">Execute</button>
        <button id="moveit">Plan & Execute</button>
        <select id="group" name="group">
        </select>
        <div id="slider-pane">
        </div>
      </div>

      </header>
    </div>
  );
}

export default App;
