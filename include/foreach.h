#ifndef MAIN_INCLUDE_FOREACH_H_
#define MAIN_INCLUDE_FOREACH_H_

#define foreach(item, array) \
    for(uint8_t keep = 1, \
            count = 0,\
            size = sizeof (array) / sizeof *(array); \
        keep && count != size; \
        keep = !keep, count++) \
      for(item = (array) + count; keep; keep = !keep)

#endif /*MAIN_INCLUDE_FOREACH_H_*/