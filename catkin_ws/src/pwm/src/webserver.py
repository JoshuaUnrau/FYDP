# app.py
import atexit
import signal
import dash
import time
from dash import dcc
from dash import html
from dash.dependencies import Input, Output
import plotly.graph_objs as go
from flask import Flask, request
import random
from geometry_msgs.msg import Pose, TransformStamped, Point
from std_msgs.msg import Float64, Int32
import threading

class DroneDashboard:

    def __init__(self, host, port):
        self.data = {"time": []}

        self.frameTime = 1000 #millis
        self.last_update = time.time()
        atexit.register(self.terminate_dashboard)
        self.reset_timer()
        self.server = Flask(__name__)
        self.app = dash.Dash(__name__, server=self.server)
        self.host = host
        self.port = port
        self.setup_layout()
        self.setup_callbacks()# Catch termination signals
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, signal, frame):
        print(f"Received signal {signal}, shutting down...")
        self.terminate_dashboard()
        exit(0)

    def setup_layout(self):
        #print("Setting layout")
        # Dynamically create options for dropdown from keys in the self.data dictionary
        # Dynamically create options for dropdown from keys in the self.data dictionary
        data_types = [key.replace("_graph", "") for key in self.data if "_graph" in key]
        options = [{'label': data_type.capitalize(), 'value': data_type} for data_type in data_types]

        self.app.layout = html.Div([
            dcc.Dropdown(
                id='data-type-dropdown',
                options=options,
                value=[option['value'] for option in options],
                multi=True
            ),
            html.Button('Pause', id='pause-button', n_clicks=0),
            html.Div(id='pause-output', children='Running...'),
            dcc.Slider(
                id='data-points-slider',
                min=10,
                max=1000,
                step=10,
                value=500,
                marks={i: str(i) for i in range(10, 1001, 100)},
                tooltip={'placement': 'bottom'}
            ),
            dcc.Graph(id='live-update-graph'),
            dcc.Interval(
                id='interval-component',
                interval=self.frameTime,  # in milliseconds
                n_intervals=0,
                max_intervals=-1  # Initially, allow unlimited intervals
            )
        ])

    def update_data(self, data_dict):
        # At the end of the update_data method, reset the timer
        self.reset_timer()

        # Update time
        if len(self.data['time']) == 0:
            self.data['time'].append(1)
        else:
            self.data['time'].append(self.data['time'][-1] + 1)

        for key, value in data_dict.items():
            graph_key = f"{key}_graph"
            if graph_key not in self.data:
                self.data[graph_key] = []
                self.setup_layout()
            self.data[graph_key].append(value)

        # General data update from data_dict
        for key in self.data:
            if key != 'time':  # We've already handled time above
                if key in data_dict:
                    self.data[key].append(data_dict[key])
                elif self.data[key]:
                    # If the key isn't in the new data_dict, carry over the last value.
                    # But only if there is a previous value to carry over.
                    self.data[key].append(self.data[key][-1])
                else:
                    # If no previous data exists for this key, just append a default value (0 or equivalent).
                    default_val = 0 if not isinstance(self.data[key], list) or len(self.data[key]) == 0 else \
                    self.data[key][-1]
                    self.data[key].append(default_val)

        #print(data_dict)
        # print(self.data)

        # Limit data history
        max_history = 500
        if len(self.data['time']) > max_history:
            for key, value in self.data.items():
                if isinstance(value, list) and value:
                    self.data[key] = value[-max_history:]

        self.app.layout['interval-component'].n_intervals += 1

    def reset_timer(self):
        # Cancel the existing timer if it's still active
        if hasattr(self, '_timer') and self._timer:
            self._timer.cancel()

        # Start a new timer with a 5-second timeout
        self._timer = threading.Timer(5, self.terminate_dashboard)
        self._timer.start()

    def terminate_dashboard(self):
        print("Killing dashboard")
        func = request.environ.get('werkzeug.server.shutdown')
        if func is None:
            raise RuntimeError('Not running with the Werkzeug Server')
        func()

    def update_graph_live(self, selected_data_types=None, max_data_points=500):
        #print("Updating graph")
        traces = []

        if selected_data_types is None:
            selected_data_types = [option['value'] for option in self.app.layout['data-type-dropdown']['options']]

        # Generic trace generation
        for data_type in selected_data_types:
            for key in self.data:
                if data_type in key and "_graph" in key:
                    label = key.replace("_graph", "").replace("_", " ").title()
                    traces.append(
                        go.Scatter(x=self.data['time'], y=self.data[key], mode='lines', name=label)
                    )

        # Limit data history based on the provided value
        while len(self.data['time']) > max_data_points:
            for key in self.data:
                self.data[key].pop(0)

        layout = go.Layout(
            title='Drone Flight Data',
            xaxis=dict(title='Time'),
            yaxis=dict(title='Values'),
        )

        return {"data": traces, "layout": layout}

    def setup_callbacks(self):
        @self.server.route('/shutdown', methods=['POST'])
        def shutdown():
            print("Shutting down gracefully...")
            func = request.environ.get('werkzeug.server.shutdown')
            if func is None:
                raise RuntimeError('Not running with the Werkzeug Server')
            func()
            return 'Server shutting down...'

        @self.app.callback(
            [Output('interval-component', 'max_intervals'),
             Output('pause-output', 'children'),
             Output('pause-button', 'children')],
            [Input('pause-button', 'n_clicks')]
        )
        def update_pause_button(n_clicks):
            if n_clicks % 2 == 0:
                return -1, 'Running...', 'Pause'
            else:
                return 0, 'Paused', 'Resume'

        @self.app.callback(
            Output('live-update-graph', 'figure'),
            [Input('interval-component', 'n_intervals'),
             Input('data-type-dropdown', 'value'),
             Input('data-points-slider', 'value')]
        )
        # def update_graph_live(n, selected_data_types, max_data_points):
        #     try:
        #         print("Callback triggered")
        #         return go.Figure(data=[go.Scatter(x=[1, 2, 3], y=[1, 2, 3])])
        #     except Exception as e:
        #         print(f"Error in update_graph_live: {e}")

        def update_graph_live(n, selected_data_types, max_data_points):
            print("Updating graph")
            traces = []

            self.setup_layout()

            # Generic trace generation
            for data_type in selected_data_types:
                for key in self.data:
                    if data_type in key and "_graph" in key:
                        label = key.replace("_graph", "").replace("_", " ").title()
                        traces.append(
                            go.Scatter(x=self.data['time'], y=self.data[key], mode='lines', name=label)
                        )

            # Limit data history based on the slider value
            while len(self.data['time']) > max_data_points:
                for key in self.data:
                    self.data[key].pop(0)

            layout = go.Layout(
                title='Drone Flight Data',
                xaxis=dict(title='Time'),
                yaxis=dict(title='Values'),
            )

            return {"data": traces, "layout": layout}

    def run(self):
        print("Server running")
        self.server.run(host=self.host, port=self.port)