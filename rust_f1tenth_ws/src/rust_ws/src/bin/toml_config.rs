use serde::Deserialize;
use std::{env, fs};

#[derive(Deserialize, Debug)]
struct Config {
    app_name: String,
    port: u16,
    debug: bool,
    db_host: String,
    db_port: u16,
    max_connections: u32,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config_path = env::var("CONFIG_PATH").unwrap_or_else(|_| "config.toml".to_string());
    let config_str = fs::read_to_string(&config_path)?;
    let config: Config = toml::from_str(&config_str)?;

    println!("{} Start!", config.app_name);
    println!("Server Port: {}", config.port);
    println!("Debug Mode: {}", config.debug);
    println!("DB: {}:{}", config.db_host, config.db_port);
    println!("Maximum connection: {}", config.max_connections);

    if config.debug {
        println!("printout debug info");
    }

    Ok(())
}
