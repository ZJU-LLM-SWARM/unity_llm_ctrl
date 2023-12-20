from cache import DiskCache
from library import search, track, simple_formation

cache = DiskCache("cache",True)
cache._load_cache()
gptcache = cache['gpt-4']
exec(gptcache)
