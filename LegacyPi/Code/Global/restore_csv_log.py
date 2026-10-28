"""Restore a downloaded .csv.gz or .csv.gz.parts/manifest.json to the original CSV."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


def restore(source, output=None):
    source = Path(source)
    with tempfile.TemporaryDirectory() as temp:
        archive = source
        if source.name == 'manifest.json':
            manifest = json.loads(source.read_text())
            filename = manifest['filename']
            if Path(filename).name != filename or manifest['encoding'] != 'gzip':
                raise ValueError('Invalid backup manifest')
            output = Path(output) if output else source.parent.parent / filename
            archive = Path(temp) / 'backup.gz'
            digest = hashlib.sha256()
            with archive.open('wb') as dest:
                for part in manifest['parts']:
                    if Path(part['name']).name != part['name']:
                        raise ValueError('Invalid backup part name')
                    part_digest, size = hashlib.sha256(), 0
                    with (source.parent / part['name']).open('rb') as stream:
                        while data := stream.read(1024 * 1024):
                            dest.write(data)
                            digest.update(data)
                            part_digest.update(data)
                            size += len(data)
                    if size != part['bytes'] or part_digest.hexdigest() != part['sha256']:
                        raise ValueError('Backup part is incomplete or corrupt: ' + part['name'])
            if digest.hexdigest() != manifest['archive_sha256']:
                raise ValueError('Backup archive checksum mismatch')
        else:
            if source.suffix != '.gz':
                raise ValueError('Expected a .csv.gz file or a parts manifest.json')
            output = Path(output) if output else source.with_suffix('')
        # Exclusive creation protects any existing original CSV.
        with output.open('xb') as dest:
            try:
                with gzip.open(archive, 'rb') as stream:
                    shutil.copyfileobj(stream, dest, length=1024 * 1024)
            except BaseException:
                output.unlink()
                raise
        return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    print(restore(args.source, args.output).resolve())
