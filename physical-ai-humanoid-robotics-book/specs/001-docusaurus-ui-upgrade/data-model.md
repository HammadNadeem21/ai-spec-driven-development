# Data Model: Docusaurus UI Upgrade

## Documentation Page
Represents individual documentation content units with metadata, content, and navigation relationships.

**Fields**:
- id: Unique identifier for the page
- title: Display title of the page
- content: Markdown content of the page
- sidebar_label: Label used in sidebar navigation
- sidebar_position: Order position in sidebar
- description: Meta description for SEO
- keywords: Array of keywords for search
- toc: Table of contents configuration
- custom_edit_url: Custom URL for edit button (if applicable)

**Relationships**:
- belongs_to: Navigation structure (parent category)
- has_many: Code blocks, images, embedded media
- next_page: Next page in navigation sequence
- previous_page: Previous page in navigation sequence

**Validation Rules**:
- title must be present and less than 100 characters
- content must be valid Markdown
- sidebar_position must be a number
- id must be unique across all pages

## Navigation Structure
Represents the hierarchical organization of documentation content for user navigation.

**Fields**:
- id: Unique identifier for the navigation item
- label: Display label for the navigation item
- type: Type of navigation item (category, link, doc)
- items: Array of child navigation items (for categories)
- link: Target URL for link type items
- docId: Reference to documentation page for doc type items
- collapsed: Boolean indicating if category is collapsed by default
- collapsible: Boolean indicating if category can be collapsed

**Relationships**:
- parent: Parent navigation item (null for root items)
- children: Array of child navigation items
- pages: Array of documentation pages within this structure

**Validation Rules**:
- label must be present
- type must be one of: 'category', 'link', 'doc'
- if type is 'category', items array must be present
- if type is 'doc', docId must reference an existing documentation page

## UI Theme
Represents the visual styling elements (colors, fonts, spacing) applied to the documentation site.

**Fields**:
- id: Unique identifier for the theme
- name: Display name of the theme
- primary_color: Main brand color in hex format
- secondary_color: Secondary color in hex format
- text_color: Primary text color in hex format
- background_color: Main background color in hex format
- font_family: CSS font family string
- font_size_base: Base font size in rem
- spacing_unit: Base spacing unit in rem
- border_radius: Default border radius in px
- breakpoints: Object containing responsive breakpoints
- custom_properties: Additional CSS custom properties

**Relationships**:
- pages: Array of pages that use this theme
- navigation: Navigation structures that use this theme

**Validation Rules**:
- All color values must be valid hex colors (#RRGGBB or #RGB)
- font_size_base must be a positive number
- spacing_unit must be a positive number
- border_radius must be a non-negative number
- breakpoints must be an object with numeric values

## Search Index
Represents the search functionality data structure for improved navigation.

**Fields**:
- id: Unique identifier for the search index entry
- title: Title of the indexed content
- content: Processed content for search indexing
- url: URL to the indexed content
- category: Category of the content
- tags: Array of tags associated with the content
- last_updated: Timestamp of last update
- relevance_score: Computed relevance score

**Relationships**:
- page: Reference to the documentation page being indexed
- navigation: Reference to the navigation structure containing the page

**Validation Rules**:
- title must be present
- url must be a valid relative or absolute URL
- relevance_score must be between 0 and 1

## Accessibility Configuration
Represents accessibility settings and compliance requirements.

**Fields**:
- id: Unique identifier for accessibility configuration
- wcag_level: WCAG compliance level (A, AA, AAA)
- high_contrast_mode: Boolean indicating if high contrast mode is supported
- screen_reader_support: Boolean indicating screen reader compatibility
- keyboard_navigation: Boolean indicating full keyboard navigation support
- alt_text_required: Boolean indicating if alt text is required for images
- color_blind_friendly: Boolean indicating color blind friendly design
- focus_indicators: Boolean indicating visible focus indicators

**Relationships**:
- pages: Array of pages that comply with this configuration
- theme: Reference to the UI theme implementing accessibility features

**Validation Rules**:
- wcag_level must be one of: 'A', 'AA', 'AAA'
- All boolean fields must be true for WCAG 2.1 AA compliance