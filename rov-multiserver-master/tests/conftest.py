"""Pytest configuration file."""
import pytest
import sys
import asyncio
import concurrent.futures

if sys.version_info[1] > 6:
    asyncio_all_tasks = asyncio.all_tasks
    asyncio_current_task = asyncio.current_task
else:
    asyncio_all_tasks = asyncio.Task.all_tasks
    asyncio_current_task = asyncio.Task.current_task


@pytest.fixture
def event_loop():
    """TODO: Document."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    yield loop

    if loop.is_closed():
        return

    async def cancel_remaining_tasks():
        all_tasks = [task for task in asyncio_all_tasks(loop=loop)
                     if task is not asyncio_current_task(loop=loop)]
        cancelling_task = asyncio.gather(*all_tasks)
        try:
            await asyncio.wait_for(cancelling_task, timeout=0.5)
        except asyncio.TimeoutError:
            pass
    if loop.is_running():
        loop.call_soon_threadsafe(loop.stop)
        while loop.is_running():
            pass
    if len(asyncio_all_tasks(loop=loop)) > 0:
        try:
            loop.run_until_complete(cancel_remaining_tasks())
        except concurrent.futures.CancelledError:
            pass
        except asyncio.CancelledError:
            pass
    if not loop.is_closed():
        loop.close()


@pytest.fixture
def threaded_event_loop(event_loop):
    """TODO: Document."""
    event_loop.run_in_executor(None, event_loop.run_forever)
    return event_loop


@pytest.fixture
def executor_pool():
    """TODO: Document."""
    pool = concurrent.futures.ThreadPoolExecutor(max_workers=20)
    yield pool
    pool.shutdown()
