from tools import logger

def get_argument(args, arg_name):
    try:
        return args[arg_name]
    except (KeyError, IndexError):
        logger.log("Key not found: " + str(arg_name), "Arg Fetcher", logger.WARNING)
        return None