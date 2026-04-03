use bevy::prelude::*;

#[inline]
pub fn is_descendant_of(
    parent: Entity,
    maybe_descendant: Entity,
    hierarchy_query: Query<&ChildOf>,
) -> bool {
    hierarchy_query
        .iter_ancestors(maybe_descendant)
        .any(|a| a == parent)
}

#[inline]
pub fn find_parent_with_component<T: Component>(
    child: Entity,
    component_query: &Query<&T>,
    hierarchy_query: &Query<&ChildOf>,
) -> Option<Entity> {
    hierarchy_query
        .iter_ancestors(child)
        .find(|a| component_query.contains(*a))
}
