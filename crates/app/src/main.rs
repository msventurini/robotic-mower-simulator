use simulation_worker::{ simulation };
// use gui_app::{GrassMap, PathMode, Robot, RobotState, Vector2};

// fn main() {
//     // Create a grass map from an image file
//     // - Cell size: 0.01m (1cm)
//     // - Simulation ticks: 10 per second
//     // - Growth rate: 8mm per day (8.0 / 24.0 / 60.0 / 60.0 mm per second)
    

//     // Run simulation for 1 day (in seconds), saving every 600 seconds (10 minutes)
//     simulation::start();
// }

use iced::Center;
use iced::widget::{Column ,button, column, text};

use iced::Result as AppResult;

fn main() -> AppResult {
    iced::run("My App",Counter::update, Counter::view)
}

#[derive(Default)]
struct Counter {
    value: i64
}

impl Counter {
    fn update(&mut self, message: Message) {
        match message {
            Message::Increment => {
                self.value += 1;
                simulation::start();
            }
            Message::Decrement => {
                self.value -= 1;
            }
        }
    }

    fn view(&self) -> Column<'_, Message> {
        column![
            button("Increment").on_press(Message::Increment),
            text(self.value).size(50),
            button("Decrement").on_press(Message::Decrement)
        ]
        .padding(20)
        .align_x(Center)
    }
}


#[derive(Debug, Clone, Copy)]
enum Message {
    Increment,
    Decrement
}