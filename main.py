import jaco2
import data
import asyncio
import signal
import sys

def stop_handler(signal, frame):
    print("\nStopping program...")
    for task in asyncio.all_tasks():
        task.cancel()
    jaco2.close_robot()
    sys.exit(0)

signal.signal(signal.SIGINT, stop_handler)

async def update_data():
    print("Starting async data update")
    while True:
        data.get_data()
        await asyncio.sleep(0.05)

async def main():
    # Main loop
    while True:
        await asyncio.sleep(1)

async def sim():
    # simulation loop
    while True:
        await asyncio.sleep(1)


if __name__ == '__main__':
    try:
        loop = asyncio.get_event_loop()
        jaco2.init_robot()
        data_updater = loop.create_task(update_data())
        main_function = loop.create_task(main())
        sim_function = loop.create_task(sim())
        loop.run_until_complete(asyncio.gather(main_function, data_updater, sim_function))
    except KeyboardInterrupt:
        stop_handler(None, None)


