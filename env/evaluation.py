import time
import gymnasium as gym
from stable_baselines3 import SAC

# Import your custom environment
from env import ContinuumEnv 

def main():
    # --- Configuration ---
    # Path to your saved model
    MODEL_PATH = "sb3_logs/sac_continuum_final.zip"
    NUM_EPISODES = 10
    
    # --- 1. Load Model and Create Env ---
    
    # Create a *single* environment, this time with render_mode="human"
    print("Creating environment with rendering...")
    env = ContinuumEnv(render_mode="human")
    
    # Load the trained agent
    try:
        model = SAC.load(MODEL_PATH, env=env)
        print(f"Model loaded from {MODEL_PATH}")
    except FileNotFoundError:
        print(f"Error: Could not find model at {MODEL_PATH}")
        print("Please make sure you have trained the model and the path is correct.")
        env.close()
        return
    except Exception as e:
        print(f"Error loading model: {e}")
        env.close()
        return
        
    print("--- Starting Visual Evaluation ---")

    # --- 2. Run Evaluation Loop ---
    for ep in range(NUM_EPISODES):
        obs, info = env.reset()
        print(f"--- Starting Episode {ep + 1}/{NUM_EPISODES} ---")
        
        terminated = False
        truncated = False
        episode_reward = 0
        step = 0
        
        while not (terminated or truncated):
            # Get action from the model (deterministic=True for best action)
            action, _states = model.predict(obs, deterministic=True)
            
            # Take the step in the environment
            obs, reward, terminated, truncated, info = env.step(action)
            
            # Manually call the render function
            # Your env's render() handles the PyVista updates
            env.render() 
            
            episode_reward += reward
            step += 1
            
            # Optional: Slow down for better visualization
            time.sleep(1/60) # ~60 FPS
            
            if terminated:
                print(f"Episode Finished: Goal Reached!")
            if truncated:
                print(f"Episode Finished: Max steps reached.")

        print(f"Episode Reward: {episode_reward:.2f} in {step} steps")
        time.sleep(1.5) # Pause between episodes

    print("--- Evaluation Complete ---")
    env.close()

if __name__ == "__main__":
    main()