#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ttf2mesh.h>

void write_glyphs(const char* ttf_file, const char* font_name, FILE* fp) {
    ttf_t* ttf;
    int res = ttf_load_from_file(ttf_file, &ttf, false);
    if (res != TTF_DONE) {
        printf("Failed to load font %s\n", ttf_file);
        exit(-1);
    }

    int next = 1;
    fwrite(&next, sizeof(int), 1, fp);
    int font_name_len = strlen(font_name);
    fwrite(&font_name_len, sizeof(int), 1, fp);
    for (int i = 0; i < font_name_len; ++i) {
        fwrite(&font_name[i], sizeof(char), 1, fp);
    }

    for (int i = 0; i < ttf->nchars; i++) {
        ttf_glyph_t *glyph = ttf->glyphs + ttf->char2glyph[i];
        if (glyph->outline == NULL || glyph->outline->total_points < 3) {
            continue;
        }

        ttf_mesh_t *mesh;
        ttf_glyph2mesh(glyph, &mesh, 128, 0);
        if (mesh == NULL) {
            continue;
        }

        ttf_outline_t* outline = mesh->outline;

        fwrite(&next, sizeof(int), 1, fp);
        fwrite(&ttf->chars[i], sizeof(unsigned int), 1, fp);
        fwrite(&outline->ncontours, sizeof(int), 1, fp);
        for (int i = 0; i < outline->ncontours; ++i) {
            fwrite(&outline->cont[i].length, sizeof(int), 1, fp);
            fwrite(&outline->cont[i].subglyph_order, sizeof(int), 1, fp);
            for (int j = 0; j < outline->cont[i].length; ++j) {
                fwrite(&outline->cont[i].pt[j].x, sizeof(float), 1, fp);
                fwrite(&outline->cont[i].pt[j].y, sizeof(float), 1, fp);
            }
        }

        fwrite(&mesh->nfaces, sizeof(int), 1, fp);
        for (int j = 0; j < mesh->nfaces; ++j) {
            fwrite(&mesh->vert[mesh->faces[j].v1].x, sizeof(float), 1, fp);
            fwrite(&mesh->vert[mesh->faces[j].v1].y, sizeof(float), 1, fp);
            fwrite(&mesh->vert[mesh->faces[j].v2].x, sizeof(float), 1, fp);
            fwrite(&mesh->vert[mesh->faces[j].v2].y, sizeof(float), 1, fp);
            fwrite(&mesh->vert[mesh->faces[j].v3].x, sizeof(float), 1, fp);
            fwrite(&mesh->vert[mesh->faces[j].v3].y, sizeof(float), 1, fp);
        }
    }

    next = 0;
    fwrite(&next, sizeof(int), 1, fp);
}

int main() {
    FILE* fp = fopen(TEST_DIR "test_data.bin", "w");

    write_glyphs(FONT_DIR "FiraSans-Bold.ttf", "FiraSans-Bold", fp);
    write_glyphs(FONT_DIR "xano/XANO-mincho-U32.ttf", "XANO-mincho-U32", fp);

    int next = 0;
    fwrite(&next, sizeof(int), 1, fp);

    fclose(fp);
}
