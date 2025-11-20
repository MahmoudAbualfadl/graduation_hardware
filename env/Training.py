import os
from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import DummyVecEnv
from gymnasium.utils.env_checker import check_env

# Import your custom environment
from env import ContinuumEnv

def main():
    # --- Configuration ---
    LOG_DIR = "sb3_logs"
    TENSORBOARD_LOG = os.path.join(LOG_DIR, "tensorboard")
    CHECKPOINT_DIR = os.path.join(LOG_DIR, "checkpoints")
    MODEL_NAME = "sac_continuum"
    
    # Training parameters
    # 1e6 is a good start for a complex problem
    TOTAL_TIMESTEPS = 1_000_000 
    CHECKPOINT_FREQ = 50_000 # Save a checkpoint every 50k steps
    
    os.makedirs(LOG_DIR, exist_ok=True)
    os.makedirs(TENSORBOARD_LOG, exist_ok=True)
    os.makedirs(CHECKPOINT_DIR, exist_ok=True)

    # --- 1. Create and Check the Environment ---
    
    # First, create a single instance to check it
    # We use render_mode=None for checking, as it's faster
    print("Checking environment...")
    check_env(ContinuumEnv(render_mode=None))
    print("Environment check passed!")

    # Create a vectorized environment for training
    # IMPORTANT: Set render_mode=None for training to make it much faster.
    # We will only render during evaluation.
    vec_env = make_vec_env(
        ContinuumEnv, 
        n_envs=1, 
        vec_env_cls=DummyVecEnv,
        env_kwargs=dict(render_mode=None)
    )

    # --- 2. Create the Callback ---
    # This saves a checkpoint of the model periodically
    checkpoint_callback = CheckpointCallback(
        save_freq=max(CHECKPOINT_FREQ // 1, 1), # n_envs=1
        save_path=CHECKPOINT_DIR,
        name_prefix=MODEL_NAME,
        save_replay_buffer=True,
        save_vecnormalize=True,
    )

    # --- 3. Create the SAC Model ---
    # SAC is designed for continuous action spaces, perfect for your env.
    model = SAC(
        "MlpPolicy", 
        vec_env, 
        verbose=1, 
        tensorboard_log=TENSORBOARD_LOG,
        buffer_size=200_000,      # Size of the replay buffer
        learning_rate=3e-4,       # Standard SAC learning rate
        batch_size=256,
        gamma=0.99,               # Discount factor
        tau=0.005,                # Soft update coefficient
        ent_coef='auto',          # Entropy regularization
        learning_starts=10000,    # Steps before starting to update policy
        policy_kwargs=dict(net_arch=[256, 256]) # Neural network architecture
    )

    # --- 4. Train the Model ---
    print(f"--- Starting Training for {TOTAL_TIMESTEPS} timesteps ---")
    print(f"Logging to {TENSORBOARD_LOG}")
    
    model.learn(
        total_timesteps=TOTAL_TIMESTEPS,
        callback=checkpoint_callback,
        log_interval=10 # Log training stats every 10 episodes
    )
    
    print("--- Training Complete ---")

    # --- 5. Save the Final Model ---
    final_model_path = os.path.join(LOG_DIR, f"{MODEL_NAME}_final")
    model.save(final_model_path)
    print(f"Final model saved to {final_model_path}.zip")

    # --- 6. Close the environment ---
    vec_env.close()
    
    print("\n--- To view training progress, run: ---")
    print(f"tensorboard --logdir={TENSORBOARD_LOG}")

if __name__ == "__main__":
    main()