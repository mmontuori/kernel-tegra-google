#include <linux/pagemap.h>
#include <linux/blkdev.h>

#define PART_NAME_SIZE 128

/*
 * add_gd_partition adds a partitions details to the devices partition
 * description.
 */
struct parsed_partitions {
	char name[BDEVNAME_SIZE];
	struct {
		sector_t from;
		sector_t size;
		int flags;
		char name[PART_NAME_SIZE];
	} parts[DISK_MAX_PARTS];
	int next;
	int limit;
};


void put_named_partition(struct parsed_partitions *p, int n, sector_t from,
	sector_t size, const char *name, size_t name_size);

static inline void
put_partition(struct parsed_partitions *p, int n, sector_t from, sector_t size)
{
	put_named_partition(p, n, from, size, NULL, 0);
}

extern int warn_no_part;

