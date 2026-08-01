from datetime import datetime

def get_current_timestamp_iso8601(dt=None):
    """
    Returns the current time in ISO 8601 format (YYYY-MM-DDTHH:MM:SS.sss)
    If no dt is provided, uses current time.
    """
    if dt is None:
        dt = datetime.now()
    return dt.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
